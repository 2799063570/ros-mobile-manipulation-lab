#!/usr/bin/env python3
"""Loopback-only, observation-only OpenVLA service; no ROS imports."""
import argparse
import base64
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODEL = 'openvla/openvla-7b'
REVISION = '47a0ec7fc4ec123775a391911046cf33cf9ed83f'


def validate_action(action):
    if len(action) != 7 or not all(math.isfinite(float(x)) for x in action):
        raise ValueError('Expected seven finite action values')
    return [float(x) for x in action]


class Predictor:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.metadata = dict(model=MODEL, revision=args.revision, unnorm_key=args.unnorm_key,
                             mock=args.mock, observation_only=True, attention='eager',
                             quantization='nf4', compute_dtype='bfloat16')
        if args.mock:
            return
        import torch
        from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig
        self.torch = torch
        torch.cuda.reset_peak_memory_stats()
        self.processor = AutoProcessor.from_pretrained(MODEL, revision=args.revision,
                                                       trust_remote_code=True)
        self.model = AutoModelForVision2Seq.from_pretrained(
            MODEL, revision=args.revision, trust_remote_code=True,
            attn_implementation='eager', torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True, device_map={'': 0},
            quantization_config=BitsAndBytesConfig(load_in_4bit=True,
                bnb_4bit_quant_type='nf4', bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_use_double_quant=True))
        self.model.eval()
        if args.unnorm_key not in self.model.norm_stats:
            raise ValueError('Unknown action statistics key: ' + args.unnorm_key)
        self.metadata.update(torch=torch.__version__, cuda=torch.version.cuda,
                             gpu=torch.cuda.get_device_name(0),
                             model_dtype=str(self.model.dtype),
                             load_peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20)

    def predict(self, payload):
        from PIL import Image
        request_id = payload['request_id']
        stamp = payload['stamp']
        if not isinstance(request_id, str) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            raise ValueError('Invalid request_id or stamp')
        instruction = payload['instruction']
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 1000:
            raise ValueError('Invalid instruction')
        raw = base64.b64decode(payload['image_png'], validate=True)
        with Image.open(io.BytesIO(raw)) as source:
            if source.width * source.height > 4096 * 4096:
                raise ValueError('Image too large')
            rgb = source.convert('RGB')
        if not self.lock.acquire(blocking=False):
            raise BlockingIOError('Inference already in flight')
        try:
            start = time.monotonic()
            if self.args.mock:
                action, memory = [0.] * 7, {}
            else:
                torch = self.torch
                prompt = 'In: What action should the robot take to ' + instruction.lower().strip() + '?\nOut:'
                inputs = self.processor(prompt, rgb).to('cuda:0', dtype=torch.bfloat16)
                # Pinned upstream predict_action appends token 29871 but does not
                # extend attention_mask. Append both here so eager attention sees
                # equal token/mask lengths and upstream does not append twice.
                if inputs['input_ids'][0, -1].item() != 29871:
                    inputs['input_ids'] = torch.cat((inputs['input_ids'],
                        inputs['input_ids'].new_full((1, 1), 29871)), dim=1)
                    inputs['attention_mask'] = torch.cat((inputs['attention_mask'],
                        inputs['attention_mask'].new_ones((1, 1))), dim=1)
                torch.cuda.synchronize()
                torch.cuda.reset_peak_memory_stats()
                start = time.monotonic()
                with torch.inference_mode():
                    action = self.model.predict_action(**inputs, unnorm_key=self.args.unnorm_key,
                                                       do_sample=False)
                torch.cuda.synchronize()
                memory = dict(peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                              reserved_mib=torch.cuda.memory_reserved() / 2**20)
            result = dict(self.metadata, request_id=request_id, stamp=stamp,
                          frame_id=payload.get('frame_id', ''), action=validate_action(action),
                          inference_ms=(time.monotonic()-start)*1000, **memory)
            return result
        finally:
            self.lock.release()


def handler_for(predictor):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def reply(self, status, payload):
            data = json.dumps(payload, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            self.reply(200 if self.path == '/health' else 404,
                       dict(predictor.metadata, ready=True) if self.path == '/health' else {'error': 'not found'})

        def do_POST(self):
            if self.path != '/predict':
                self.reply(404, {'error': 'not found'})
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 8 * 1024 * 1024:
                    raise ValueError('Invalid body length')
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError('Expected JSON object')
                result = predictor.predict(payload)
                self.reply(200, result)
            except BlockingIOError as exc:
                self.reply(409, {'error': str(exc)})
            except (ValueError, KeyError, TypeError, OSError) as exc:
                self.reply(400, {'error': str(exc)})
            except Exception as exc:
                self.log_error('Inference failed: %r', exc)
                self.reply(500, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8008)
    parser.add_argument('--revision', default=REVISION)
    parser.add_argument('--unnorm-key', default='bridge_orig')
    parser.add_argument('--mock', action='store_true', help='Transport tests only; explicitly marked mock')
    args = parser.parse_args()
    predictor = Predictor(args)
    print(json.dumps(dict(predictor.metadata, port=args.port)), flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(predictor)).serve_forever()


if __name__ == '__main__':
    main()
