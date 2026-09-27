#!/usr/bin/env python3
"""Resume the pinned snapshot, then record a real GPU inference acceptance run."""
import argparse
import base64
import json
import os
from pathlib import Path
import statistics
import time
import traceback

# Xet TLS does not work on this machine's current network. HTTP supports resume.
os.environ.setdefault('HF_HUB_DISABLE_XET', '1')
os.environ.setdefault('HF_HUB_DOWNLOAD_TIMEOUT', '60')
from huggingface_hub import snapshot_download
from model_service import MODEL, REVISION, Predictor


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', type=Path, default=Path(__file__).resolve().parents[1]/'validation/gazebo_rgb.png')
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'validation/real_model.json')
    p.add_argument('--count', type=int, default=5)
    args = p.parse_args()
    if args.count < 1:
        p.error('--count must be positive')
    status = {'model': MODEL, 'revision': REVISION, 'stage': 'download', 'passed': False}
    def save():
        args.output.parent.mkdir(parents=True, exist_ok=True)
        tmp = args.output.with_suffix('.tmp')
        tmp.write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')
        tmp.replace(args.output)
        print(json.dumps(status), flush=True)
    save()
    try:
        for attempt in range(1, 4):
            status['download_attempt'] = attempt
            save()
            try:
                status['snapshot'] = snapshot_download(MODEL, revision=REVISION,
                    allow_patterns=['*.json', '*.py', '*.model', '*.safetensors'], max_workers=3)
                break
            except Exception:
                if attempt == 3:
                    raise
                traceback.print_exc()
                time.sleep(10)
        status['stage'] = 'load'
        save()
        predictor = Predictor(argparse.Namespace(mock=False, revision=REVISION, unnorm_key='bridge_orig'))
        png = base64.b64encode(args.image.read_bytes()).decode('ascii')
        results = []
        status.update(stage='inference', results=results)
        for i in range(args.count+1):
            result = predictor.predict(dict(image_png=png, request_id='acceptance-%d' % i,
                stamp=0.0, frame_id='saved_gazebo_rgb', instruction='pick up the red block'))
            result['warmup'] = i == 0
            results.append(result)
            save()
        timed = results[1:]
        status.update(stage='complete', passed=True,
            median_inference_ms=statistics.median(r['inference_ms'] for r in timed),
            peak_allocated_mib=max(r['peak_allocated_mib'] for r in timed))
        save()
    except Exception as exc:
        status.update(failed_stage=status['stage'], stage='failed', error=str(exc))
        save()
        traceback.print_exc()
        raise


if __name__ == '__main__':
    main()
