#!/usr/bin/env python3
"""Select model/OpenCV scripts without editing the dispatcher or firmware."""
import argparse
import os
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--backend',choices=('model','opencv'),default='model')
p.add_argument('--separator-backend',choices=('model','opencv'),help='Defaults to --backend')
p.add_argument('--model', help='Checkpoint path relative to vision_orin, or absolute; overrides model_config.json for this run')
a=p.parse_args()
env=os.environ.copy()
if a.backend == 'model' or a.separator_backend == 'model':
    # The single persistent model warms up in the dispatcher background worker.
    env.setdefault('LARC_STARTUP_GRACE_SEC', '120')
if a.model:
    if a.backend=='opencv' and (a.separator_backend or a.backend)=='opencv':
        p.error('--model requires at least one model backend')
    model_path=Path(a.model).expanduser()
    if not model_path.is_absolute(): model_path=ROOT/model_path
    if not model_path.is_file(): p.error(f'Model not found: {model_path}')
    env['LARC_MODEL_WEIGHTS']=str(model_path)
env['LARC_INTAKE_PY']=str(ROOT/'main_vision'/('orin_vision_model.py' if a.backend=='model' else 'orin_vision.py'))
env['LARC_SEPARATOR_PY']=str(ROOT/'separator'/('separator_model.py' if (a.separator_backend or a.backend)=='model' else 'separator_vision.py'))
os.chdir(ROOT)
os.execve(sys.executable,[sys.executable,'-u',str(ROOT/'dispatcher/dispatcher.py')],env)
