#!/usr/bin/env python3
"""Verify installed libraries, packaged weights, CUDA inference and saved ROIs."""
import argparse
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from model_runtime.core import Detector, load_roi, regions, settings

def main():
    import numpy as np
    import torch
    import torchvision
    from torchvision.ops import nms
    p=argparse.ArgumentParser()
    p.add_argument('--device',default='0')
    a=p.parse_args()
    device='cuda:0' if a.device=='0' else a.device
    nms(torch.tensor([[0.,0.,10.,10.]],device=device),torch.tensor([.9],device=device),.5)
    detector=Detector(settings(device=a.device))
    print('Model:',detector.options['weights'])
    print('Task:',detector.model.task,'Classes:',detector.model.names)
    print('torch:',torch.__version__,'torchvision:',torchvision.__version__)
    image=np.zeros((376,1344,3),dtype=np.uint8)
    out,_=detector.run(image,'intake',load_roi('intake'),time.monotonic())
    print('Inference succeeded. First-call warm-up:',out['inference_ms'],'ms')
    for role,(w,h) in detector.expected.items():
        try:
            print(role,'ROI:',regions(role,load_roi(role),(h,w)))
        except ValueError as exc:
            print('ACTION NEEDED:',exc)
    print('Dependencies checked. Live timing and camera/servo tests must be done on the Orin.')
if __name__=='__main__': main()
