#!/usr/bin/env python3
"""Model-based separator; emits the existing dispatcher protocol."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from model_runtime.headless import main
if __name__ == "__main__":
    raise SystemExit(main("separator"))
