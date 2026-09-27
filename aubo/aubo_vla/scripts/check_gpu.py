#!/usr/bin/env python3
import json
import torch
import bitsandbytes as bnb
assert torch.cuda.is_available()
x = torch.randn(1024, 1024, device='cuda', dtype=torch.bfloat16)
y = x @ x
assert torch.isfinite(y).all().item()
layer = bnb.nn.Linear4bit(1024, 64, bias=False, compute_dtype=torch.bfloat16,
                         quant_type='nf4', compress_statistics=True).to('cuda')
z = layer(x[:1])
torch.cuda.synchronize()
assert torch.isfinite(z).all().item()
print(json.dumps(dict(torch=torch.__version__, cuda=torch.version.cuda,
                      gpu=torch.cuda.get_device_name(0), capability=torch.cuda.get_device_capability(0),
                      bitsandbytes=bnb.__version__, matmul=True, nf4=True)))
