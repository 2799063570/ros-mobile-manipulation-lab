#!/usr/bin/env python3
"""Package source + Linux CPython 3.10 wheels. Never include local credentials."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--output', default=str(ROOT / 'build/aubo-dashboard-functiongraph.zip'))
args = parser.parse_args()
output = Path(args.output).resolve()
output.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='aubo-dashboard-build-') as temp:
    stage = Path(temp)
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
        '--only-binary=:all:', '--platform', 'manylinux2014_x86_64', '--python-version', '310',
        '--implementation', 'cp', '--abi', 'cp310', '--target', str(stage),
        '-r', str(ROOT / 'requirements.txt')], check=True)
    for name in ('app.py', 'index.py', 'requirements.txt'):
        shutil.copy2(ROOT / name, stage / name)
    shutil.copytree(ROOT / 'dist', stage / 'dist')
    # Record the exact dependency versions shipped in this artifact.
    versions = []
    for metadata in sorted(stage.glob('*.dist-info/METADATA')):
        lines = metadata.read_text().splitlines()
        name = next(line[6:] for line in lines if line.startswith('Name: '))
        version = next(line[9:] for line in lines if line.startswith('Version: '))
        versions.append(name + '==' + version)
    (stage / 'dependencies.lock').write_text('\n'.join(sorted(versions)) + '\n')
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(stage.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
                archive.write(path, path.relative_to(stage))
print('Created %s (%.1f MB) · Python 3.10 / Linux x86_64' % (output, output.stat().st_size / 1048576))
