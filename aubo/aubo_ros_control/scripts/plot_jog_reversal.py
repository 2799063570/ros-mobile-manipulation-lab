#!/usr/bin/env python3
"""Plot jog_reversal_probe CSV; a commanded joint sequence, not measured TCP motion."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    result = {}
    for group, (V, A, J) in enumerate(((.25, .6, 8), (.35, .8, 12))):
        data = [r for r in rows if r["group"] == group]
        t = [r["time"] for r in data]
        for axis, field, limit, unit in zip(axes[:, group], ("velocity", "acceleration", "jerk"), (V, A, J), ("rad/s", "rad/s^2", "rad/s^3")):
            axis.plot(t, [r[field] for r in data], label=field)
            axis.axhline(limit, color='grey', linestyle=':'); axis.axhline(-limit, color='grey', linestyle=':')
            axis.axvline(2, color='orange', linestyle='--'); axis.axvline(5, color='orange', linestyle='--')
            axis.set_ylabel(unit); axis.grid(alpha=.2); axis.legend(loc='upper right')
        axes[0, group].plot(t, [r["target"] for r in data], '--', label="desired velocity")
        axes[0, group].legend(); axes[0, group].set_title(f"Joint group {group + 1}: V={V}, A={A}, J={J}")
        axes[-1, group].set_xlabel("Nominal point sequence time (s), h=5ms")
        cross = next(r["time"] for r in data if r["time"] >= 2 and r["velocity"] >= 0)
        result[f"group_{group}"] = dict(V=V, A=A, J=J, alpha=J*.005/(2*A),
            reversal_to_zero_s=cross-2,
            continued_negative_displacement_rad=sum(-r["velocity"]*.005 for r in data if 2 <= r["time"] < cross),
            max_abs_velocity=max(abs(r['velocity']) for r in data),
            max_abs_acceleration=max(abs(r['acceleration']) for r in data),
            max_abs_jerk=max(abs(r['jerk']) for r in data))
    fig.suptitle("Production smoother: -V to +V at 2s; zero command at 5s\nCommanded joint sequence only; transport/buffer/actual tracking not included")
    fig.tight_layout(); args.output.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output / "reversal.png", dpi=160)
    (args.output / "reversal_summary.json").write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
