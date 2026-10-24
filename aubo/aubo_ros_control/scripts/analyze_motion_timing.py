#!/usr/bin/env python3
"""Read-only analysis of MotionTimingTrace CSV. Does not import ROS or send commands."""
import argparse
import bisect
import csv
import json
import statistics
from pathlib import Path


def stats(values):
    values = sorted(values)
    if not values:
        return {"n": 0}
    def quantile(p):
        x = p * (len(values) - 1)
        lo = int(x)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (x - lo)
    return dict(n=len(values), min=values[0], p50=quantile(.5), p95=quantile(.95),
                p99=quantile(.99), max=values[-1], mean=statistics.mean(values),
                std=statistics.pstdev(values))


def validate_events(rows):
    """Reject corrupt associations before reporting queue delay or derivatives.

    Begin-without-return is legitimate on a stopped capture. Generated points
    without a batch append are legitimate initial holds or rejected batches.
    """
    indexes = {}
    for kind, key in (("point", "id"), ("batch_append", "id"),
                      ("send_begin", "batch_id"), ("send_end", "batch_id"),
                      ("query_begin", "query_id"), ("query_end", "query_id")):
        events = [r for r in rows if r["kind"] == kind]
        index = {r[key]: r for r in events}
        if len(index) != len(events):
            raise ValueError(f"Duplicate {kind} {key}; timing associations are invalid")
        indexes[kind] = index
    for kind in ("point", "target", "command"):
        for r in (r for r in rows if r["kind"] == kind):
            if not r["begin_ns"] <= r["locked_ns"] <= r["end_ns"]:
                raise ValueError(f"Invalid timestamp order in {kind}")
    counts = {}
    for pid, append in indexes["batch_append"].items():
        point = indexes["point"].get(pid)
        if not point or point["batch_id"] != append["batch_id"] or point["end_ns"] > append["begin_ns"]:
            raise ValueError(f"Invalid point -> batch association for point {pid}")
        batch = append["batch_id"]
        counts[batch] = counts.get(batch, 0) + 1
        begin = indexes["send_begin"].get(batch)
        if begin and append["begin_ns"] > begin["begin_ns"]:
            raise ValueError(f"Point {pid} was appended after its SDK send began")
    for bid, begin in indexes["send_begin"].items():
        if counts.get(bid, 0) != begin["count"]:
            raise ValueError(f"Batch {bid} point count does not match SDK send count")
    for kind in ("send", "query"):
        for eid, end in indexes[kind + "_end"].items():
            begin = indexes[kind + "_begin"].get(eid)
            if not begin or begin["begin_ns"] != end["begin_ns"] or end["end_ns"] < begin["begin_ns"]:
                raise ValueError(f"Invalid {kind} begin/return association: {eid}")


def analyze(path, output):
    with path.open() as stream:
        metadata = stream.readline().strip()
        if not metadata.startswith("# motion_timing_trace_v1;"):
            raise ValueError("Incomplete or unsupported capture; no valid shutdown dump")
        rows = list(csv.DictReader(stream))
    integers = ["id", "batch_id", "target_id", "command_id", "query_id", "begin_ns",
                "locked_ns", "end_ns", "source_ros_ns", "count", "mac_scalars", "result"]
    for row in rows:
        for key in integers:
            row[key] = int(row[key])
    validate_events(rows)
    def kind(name):
        return sorted((r for r in rows if r["kind"] == name), key=lambda r: r["begin_ns"])
    sends, queries, points, targets = (kind(k) for k in ("send_end", "query_end", "point", "target"))
    begins = kind("send_begin")
    send_by_batch = {r["batch_id"]: r for r in sends}
    begin_by_batch = {r["batch_id"]: r for r in begins}
    appended = {r["id"]: r for r in kind("batch_append")}
    target_by_id = {r["target_id"]: r for r in targets}
    command_by_id = {r["command_id"]: r for r in kind("command")}
    query_by_id = {r["query_id"]: r for r in queries}
    output.mkdir(parents=True, exist_ok=True)
    lifecycle = []
    for point in points:
        batch = point["batch_id"]
        send, begin, append = send_by_batch.get(batch), begin_by_batch.get(batch), appended.get(point["id"])
        target = target_by_id.get(point["target_id"])
        command = command_by_id.get(target["command_id"]) if target else None
        lifecycle.append(dict(point_id=point["id"], batch_id=batch,
            target_id=point["target_id"], latest_command_id=point["command_id"],
            target_command_id=target["command_id"] if target else "",
            command_received_ns=command["begin_ns"] if command else "",
            target_begin_ns=target["begin_ns"] if target else "",
            generate_begin_ns=point["begin_ns"], generate_lock_ns=point["locked_ns"],
            generated_ns=point["end_ns"], queue_enqueue_ns="", queue_dequeue_ns="",
            batch_append_ns=append["begin_ns"] if append else "",
            send_begin_ns=begin["begin_ns"] if begin and append else "",
            send_end_ns=send["end_ns"] if send and append else "",
            result=send["result"] if send and append else "",
            state="returned" if send and append else "inflight" if begin and append else "not_sent"))
    def write_csv(name, data):
        if data:
            with (output / name).open("w") as f:
                writer = csv.DictWriter(f, data[0].keys())
                writer.writeheader()
                writer.writerows(data)
    write_csv("point_lifecycle.csv", lifecycle)
    batches = []
    good_queries = [q for q in queries if q["result"] == 0]
    query_starts = [q["begin_ns"] for q in good_queries]
    for i, send in enumerate(begins):
        reply = send_by_batch.get(send["batch_id"])
        query = query_by_id.get(send["query_id"])
        # This is the NEXT observed MAC after the call returns, NOT a cabinet
        # sample at send completion and not a count of acknowledged/executed IDs.
        after_index = bisect.bisect_left(query_starts, reply["end_ns"]) if reply else len(good_queries)
        after = good_queries[after_index] if after_index < len(good_queries) else None
        batches.append(dict(batch_id=send["batch_id"], points=send["count"],
            nominal_coverage_ms=send["count"] * 5,
            send_begin_ns=send["begin_ns"], send_end_ns=reply["end_ns"] if reply else "",
            rpc_ms=(reply["end_ns"] - reply["begin_ns"]) / 1e6 if reply else "",
            send_interval_ms=(send["begin_ns"] - begins[i-1]["begin_ns"]) / 1e6 if i else "",
            before_query_id=send["query_id"], before_mac_scalars=send["mac_scalars"],
            before_response_age_ms=(send["begin_ns"]-query["end_ns"]) / 1e6 if query else "",
            after_query_id=after["query_id"] if after else "",
            after_mac_scalars=after["mac_scalars"] if after else "",
            after_query_end_ns=after["end_ns"] if after else ""))
    write_csv("batches.csv", batches)
    active_queries = [q for q in queries if begins and q["begin_ns"] >= begins[0]["begin_ns"] and q["result"] == 0]
    nonzero = [q["mac_scalars"] / 6.0 * 5.0 for q in active_queries if q["mac_scalars"] > 0]
    intervals = [(b["begin_ns"]-a["begin_ns"]) / 1e6 for a, b in zip(begins, begins[1:])]
    # Differentiate the submitted absolute positions at the NOMINAL 5ms point
    # spacing, not their host-generation wall time (a batch is generated fast).
    derivative_max = {name: [0.0]*6 for name in ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")}
    previous = previous_v = previous_a = None
    derivative_samples = 0
    for point in sorted(points, key=lambda r: r["id"]):
        reply = send_by_batch.get(point["batch_id"])
        if point["id"] not in appended or not reply or reply["result"] != 0:
            previous = previous_v = previous_a = None
            continue
        q = [float(point[f"q{i}"]) for i in range(6)]
        if previous and point["id"] == previous[0]+1:
            v = [(x-y)/.005 for x,y in zip(q,previous[1])]
            a = [(x-y)/.005 for x,y in zip(v,previous_v)] if previous_v else None
            j = [(x-y)/.005 for x,y in zip(a,previous_a)] if a and previous_a else None
            for name, values in zip(derivative_max, (v,a,j)):
                if values is not None:
                    derivative_max[name] = [max(x,abs(y)) for x,y in zip(derivative_max[name],values)]
            previous_v, previous_a = v, a
            derivative_samples += 1
        else:
            previous_v = previous_a = None
        previous = point["id"], q
    summary = dict(metadata=metadata, trace_dropped_events=int(metadata.rsplit("dropped=", 1)[-1]),
        scope="on-demand jog; CommandQueue is bypassed",
        cautions=["SDK return is not physical execution; MAC has no hardware sample timestamp",
                  "MAC statistics are sample-weighted, NOT time occupancy",
                  "variable batch intervals are not fixed-period scheduling jitter",
                  "point command_id is latest received; target event gives target provenance",
                  "stop guards may override the target; point.desired records the actual input to smoothing"],
        counts={k: len(kind(k)) for k in sorted({r["kind"] for r in rows})},
        unfinished_sends=len(begins)-len(sends),
        unfinished_queries=len(kind("query_begin"))-len(queries),
        send_rpc_ms=stats([(r["end_ns"]-r["begin_ns"])/1e6 for r in sends]),
        query_rpc_ms=stats([(r["end_ns"]-r["begin_ns"])/1e6 for r in queries]),
        generation_ms=stats([(r["end_ns"]-r["begin_ns"])/1e6 for r in points]),
        generation_lock_wait_ms=stats([(r["locked_ns"]-r["begin_ns"])/1e6 for r in points]),
        target_lock_wait_ms=stats([(r["locked_ns"]-r["begin_ns"])/1e6 for r in targets]),
        target_computation_ms=stats([(r["end_ns"]-r["locked_ns"])/1e6 for r in targets]),
        generated_to_send_ms=stats([(r["send_begin_ns"]-r["generated_ns"])/1e6 for r in lifecycle if r["send_begin_ns"] != ""]),
        local_batch_wait_ms=stats([(r["send_begin_ns"]-r["batch_append_ns"])/1e6 for r in lifecycle if r["send_begin_ns"] != ""]),
        send_interval_ms=stats(intervals),
        batch_rate_hz=1000/statistics.mean(intervals) if intervals else None,
        target_interval_error_vs_20ms=stats([(b["begin_ns"]-a["begin_ns"])/1e6-20 for a,b in zip(targets,targets[1:])]),
        # A balance metric, not a timestamp of actual cabinet consumption.
        interval_minus_previous_batch_coverage_ms=stats([dt-a["count"]*5 for dt,a in zip(intervals,begins)]),
        observed_nonzero_mac_coverage_ms=stats(nonzero),
        observed_mac_zero_after_first_send=sum(q["mac_scalars"]==0 for q in active_queries),
        observed_nonzero_mac_below_100ms=sum(x < 100 for x in nonzero),
        nominal_sent_sequence_derivative_max=derivative_max,
        nominal_sent_sequence_derivative_samples=derivative_samples)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary, begins, sends, queries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    summary, begins, sends, queries = analyze(args.trace, args.output)
    if args.plot and (sends or queries):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        origin = min(r["begin_ns"] for r in sends + queries)
        t = lambda r: (r["begin_ns"] - origin) / 1e9
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for rows, label in ((sends, "send SDK call"), (queries, "MAC query")):
            axes[0].plot([t(r) for r in rows], [(r["end_ns"]-r["begin_ns"])/1e6 for r in rows], '.-', label=label)
        axes[0].set_ylabel("RPC (ms)"); axes[0].legend()
        good = [r for r in queries if r["result"] == 0]
        axes[1].plot([(r["end_ns"]-origin)/1e9 for r in good], [r["mac_scalars"] / 6 for r in good], '.-')
        axes[1].set_ylabel("Observed MAC (points)")
        axes[2].plot([t(r) for r in begins[1:]], [(b["begin_ns"]-a["begin_ns"])/1e6 for a,b in zip(begins,begins[1:])], '.-', label="send start interval")
        axes[2].plot([t(r) for r in begins], [r["count"]*5 for r in begins], '.-', label="batch nominal coverage")
        axes[2].set_ylabel("Time (ms)"); axes[2].set_xlabel("Steady elapsed time (s)"); axes[2].legend()
        for ax in axes: ax.grid(alpha=.25)
        fig.suptitle("Host trace: MAC sample time and hardware execution time unknown")
        fig.tight_layout(); fig.savefig(args.output / "timing.png", dpi=160)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
