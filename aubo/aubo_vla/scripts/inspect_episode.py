#!/usr/bin/env python3
"""Check an episode and export measured next-state deltas, without ROS/motion."""
import argparse
import json
from pathlib import Path
import sys

# Allows direct source invocation on a machine without catkin/ROS.
source = Path(__file__).resolve().parents[1]/'src'
if source.is_dir():
    sys.path.insert(0, str(source))
from aubo_vla.dataset import inspect_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    parser.add_argument('--period-tolerance', type=float, default=0.20)
    parser.add_argument('--export', type=Path, help='Create a new JSONL file; never overwrite')
    parser.add_argument('--include-failure', action='store_true')
    args = parser.parse_args()
    try:
        report, actions = inspect_episode(args.episode, args.period_tolerance)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not report['passed']:
            return 1
        if args.export:
            if report['outcome'] != 'success' and not args.include_failure:
                raise ValueError('Failure export requires --include-failure')
            with args.export.open('x', encoding='utf-8') as output:
                for action in actions:
                    action.update(episode=str(args.episode.resolve()), outcome=report['outcome'])
                    output.write(json.dumps(action, ensure_ascii=False, allow_nan=False)+'\n')
        return 0
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print('Episode check failed: '+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
