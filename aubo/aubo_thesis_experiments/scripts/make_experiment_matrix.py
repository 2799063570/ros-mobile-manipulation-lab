#!/usr/bin/env python3
"""Balanced randomized order for the four visual/planner combinations."""
import argparse
import csv
import random
from pathlib import Path

def make_matrix(scenes, seeds, repetitions, order_seed, prefix):
    if not scenes or len(set(scenes)) != len(scenes) or not seeds or len(set(seeds)) != len(seeds) or repetitions < 1:
        raise ValueError('unique scenes/seeds and positive repetitions required')
    rng, rows = random.Random(order_seed), []
    for scene in scenes:
        for seed in seeds:
            for repeat in range(1, repetitions+1):
                conditions = [('S1','YOLO11n-OBB','RRTConnectkConfigDefault',''),
                              ('S2','GC-YOLO','RRTConnectkConfigDefault',''),
                              ('S3','YOLO11n-OBB','NSG-RRT-Connect','/thesis_experiment/nsg_plan'),
                              ('S4','GC-YOLO','NSG-RRT-Connect','/thesis_experiment/nsg_plan')]
                rng.shuffle(conditions)
                for condition, vision, planner, service in conditions:
                    rows.append({'run_id': '%s_%s_seed%s_r%s_%s'%(prefix,scene,seed,repeat,condition),
                                 'pair_id': '%s_seed%s_r%s'%(scene,seed,repeat), 'scene_id':scene,
                                 'run_seed':seed, 'repeat':repeat, 'condition':condition,
                                 'vision':vision, 'planner':planner, 'planning_service':service})
    return rows

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scenes', required=True, help='comma-separated physical scene/reset IDs')
    p.add_argument('--seeds', default='42,43,44,45,46')
    p.add_argument('--repetitions', type=int, default=1)
    p.add_argument('--order-seed', type=int, default=42)
    p.add_argument('--prefix', required=True, help='unique experiment batch name')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    rows = make_matrix(a.scenes.split(','), list(map(int,a.seeds.split(','))), a.repetitions, a.order_seed, a.prefix)
    with Path(a.output).open('x', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print('%s runs written; reset scene before every row. run_seed does not set OMPL RNG automatically.'%len(rows))

if __name__ == '__main__':
    main()
