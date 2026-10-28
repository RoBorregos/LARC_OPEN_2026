#!/usr/bin/env bash
# Keep working system CUDA torch/torchvision; isolate added Python packages.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
python3 - <<'PY'
import torch, torchvision, cv2
from torchvision.ops import nms
assert torch.cuda.is_available(), 'System Python must have working CUDA PyTorch'
nms(torch.tensor([[0.,0.,10.,10.]],device='cuda'),torch.tensor([.9],device='cuda'),.5)
print('Preserving CUDA torch',torch.__version__,'torchvision',torchvision.__version__,'OpenCV',cv2.__version__)
PY
python3 -m venv --system-site-packages "$ROOT/.venv-model"
"$ROOT/.venv-model/bin/python" - "$ROOT" <<'PY'
import importlib.metadata as m
from pathlib import Path
import subprocess
import sys
root=Path(sys.argv[1])
pip=[sys.executable,'-m','pip','install']
# Do not let the resolver substitute desktop/CPU torch or another OpenCV.
subprocess.check_call(pip+['--no-deps','ultralytics==8.4.48','ultralytics-thop==2.1.6'])
protected=('torch','torchvision','numpy')
constraints=root/'.venv-model'/'preserved-packages.txt'
constraints.write_text(''.join(f'{name}=={m.version(name)}\n' for name in protected))
from pip._vendor.packaging.requirements import Requirement
requirements=['pyserial']
for text in m.requires('ultralytics') or []:
    r=Requirement(text)
    if r.marker and not r.marker.evaluate({'extra':''}): continue
    if r.name.lower().replace('_','-') in ('torch','torchvision','opencv-python','ultralytics-thop'): continue
    requirements.append(text)
subprocess.check_call(pip+['-c',str(constraints)]+requirements)
PY
"$ROOT/.venv-model/bin/python" "$ROOT/tools/check_model.py"
