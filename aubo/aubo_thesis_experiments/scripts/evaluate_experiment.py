#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
from aubo_thesis_experiments.metrics import localization, summarize_events

def main():
    p = argparse.ArgumentParser(description='Analyze recorded data; never fills missing results.')
    p.add_argument('mode', choices=['localization', 'events'])
    p.add_argument('input')
    p.add_argument('--annotations')
    p.add_argument('--angle-threshold-deg', type=float)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    if a.mode == 'localization':
        if a.angle_threshold_deg is None:
            p.error('localization requires an explicit --angle-threshold-deg')
        with open(a.input, encoding='utf-8-sig', newline='') as f:
            report = localization(list(csv.DictReader(f)), a.angle_threshold_deg)
    else:
        with open(a.input, encoding='utf-8') as f:
            events = [json.loads(line) for line in f if line.strip()]
        labels = []
        if a.annotations:
            with open(a.annotations, encoding='utf-8-sig', newline='') as f:
                labels = list(csv.DictReader(f))
        report = summarize_events(events, labels)
    Path(a.output).write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))

if __name__ == '__main__':
    main()
