#!/usr/bin/env python3
"""Benchmark a saved Gazebo RGB image against the resident service."""
import argparse
import base64
import json
import math
import statistics
import time
import urllib.request
import uuid

p = argparse.ArgumentParser()
p.add_argument('image')
p.add_argument('--url', default='http://127.0.0.1:8008/predict')
p.add_argument('--instruction', default='pick up the red block')
p.add_argument('--count', type=int, default=5)
p.add_argument('--timeout', type=float, default=120)
p.add_argument('--allow-mock', action='store_true')
a = p.parse_args()
assert a.count > 0
with open(a.image, 'rb') as f:
    image = base64.b64encode(f.read()).decode('ascii')
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
results = []
for i in range(a.count + 1):
    payload = dict(image_png=image, instruction=a.instruction, request_id=str(uuid.uuid4()),
                   stamp=0.0, frame_id='saved_gazebo_rgb')
    request = urllib.request.Request(a.url, json.dumps(payload).encode(), {'Content-Type': 'application/json'})
    start = time.monotonic()
    with opener.open(request, timeout=a.timeout) as response:
        result = json.load(response)
    assert result['request_id'] == payload['request_id']
    assert a.allow_mock or result['mock'] is False, 'Mock is not a model acceptance test'
    assert len(result['action']) == 7 and all(math.isfinite(x) for x in result['action'])
    result.update(warmup=i == 0, wall_ms=(time.monotonic()-start)*1000)
    print(json.dumps(result), flush=True)
    if i:
        results.append(result)
print(json.dumps(dict(samples=len(results), median_inference_ms=statistics.median(
    r['inference_ms'] for r in results), max_inference_ms=max(r['inference_ms'] for r in results))), flush=True)
