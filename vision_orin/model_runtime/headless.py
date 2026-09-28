"""Dispatcher-compatible headless entry point, shared by both cameras."""
import argparse
import faulthandler
import signal
import sys
import time
from model_runtime.core import Detector, load_roi, neutral, settings
from model_runtime.capture import Camera

def main(role):
    p = argparse.ArgumentParser()
    p.add_argument('--model')
    p.add_argument('--device', help='0 for Orin GPU; cpu for debugging')
    p.add_argument('--camera-device')
    args = p.parse_args()
    # Show where Python is stuck if camera setup or inference stops returning.
    faulthandler.enable()
    faulthandler.dump_traceback_later(45, repeat=False)
    camera = None
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        started = time.monotonic()
        print(f"[{role}] STARTUP: loading model/runtime (robot must wait)", file=sys.stderr, flush=True)
        detector = Detector(settings(weights=args.model, device=args.device))
        if not running:
            return 0
        print(f"[{role}] STARTUP: model loaded in {time.monotonic()-started:.1f}s; opening camera", file=sys.stderr, flush=True)
        cfg = load_roi(role)
        camera = Camera(role, args.camera_device)
        seen = 0
        last_log = last_output = 0.
        last_roi_check = 0.
        roi_error = None
        ready = False
        last_wait = last_valid_trace_reset = 0.0
        print(f"[{role}] Model loaded; waiting for first fresh inference", file=sys.stderr, flush=True)
        while running:
            frame, stamp, seq, error = camera.latest()
            if frame is None or seq == seen or error:
                if (frame is None or error or time.monotonic() - stamp > .4) and time.monotonic() - last_wait >= 2:
                    print(f"[{role}] WAITING for camera frame: {error or 'no new frame'}", file=sys.stderr, flush=True)
                    last_wait = time.monotonic()
                time.sleep(.01)
                continue
            seen = seq
            if time.monotonic() - last_roi_check >= .5:
                last_roi_check = time.monotonic()
                try:
                    cfg = load_roi(role)
                    roi_error = None
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    if roi_error != str(exc):
                        print(f'[{role}] ROI reload failed; outputs neutral: {exc}', file=sys.stderr, flush=True)
                    roi_error = str(exc)
            if roi_error is not None:
                last_output = time.monotonic()
                continue
            if not ready:
                print(f"[{role}] STARTUP: inference/warm-up at +{time.monotonic()-started:.1f}s", file=sys.stderr, flush=True)
            out, _ = detector.run(frame, role, cfg, stamp)
            if not running:
                break
            # Startup neutral messages must not end the dispatcher's startup grace.
            # A cold CUDA inference can be too old; wait for a fresh valid result.
            if not ready and not out.get('error'):
                ready = True
                print(f'[{role}] READY: first fresh inference; normal reporting started', file=sys.stderr, flush=True)
            if ready and not out.get('error'):
                print(out['message'], flush=True)
                if time.monotonic() - last_valid_trace_reset >= 2:
                    faulthandler.dump_traceback_later(45, repeat=False)
                    last_valid_trace_reset = time.monotonic()
            last_output = time.monotonic()
            if last_output-last_log >= 2:
                print(f"[{role}] infer={out['inference_ms']}ms network={out['network_ms']}ms "
                      f"decision={out['decision_ms']}ms processing={out['processing_ms']}ms frame_age={out['frame_age_ms']}ms "
                      f"interval={out['output_interval_ms']}ms detections={out['detection_count']} "
                      f"{out['error'] or 'OK'}", file=sys.stderr, flush=True)
                last_log = last_output
    except Exception as exc:
        print(f'[{role}] ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        faulthandler.cancel_dump_traceback_later()
        if camera:
            camera.close()
    return 0
