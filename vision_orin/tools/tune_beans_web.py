#!/usr/bin/env python3
"""
tune_beans_web.py — tune the BEANS and BENEFITS cameras from a browser

Purpose : Everything tune_beans.py does, served over HTTP instead of drawn
          with cv2.imshow, plus the three benefits ROIs (click to place).

Usage
    sudo systemctl stop larc-dispatcher       it owns the cameras
    cd ~/LARC_OPEN_2026/vision_orin
    python3 tools/tune_beans_web.py           then open http://localhost:8080
    python3 tools/tune_beans_web.py --no-intake --no-separator    benefits only

"""

from __future__ import annotations

import argparse
import json
import os
import socket
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(REPO / "link"))
sys.path.insert(0, str(REPO / "benefits"))

# One copy of the detection code, and it is the verified one.
from tune_beans import (                                    # noqa: E402
    INTAKE_CONFIG, INTAKE_DEFAULTS, SEPARATOR_CONFIG, SEPARATOR_DEFAULTS,
    CameraThread, clamp_roi, derive_intake, derive_separator,
    intake_detect, intake_masks, intake_trigger, load_raw,
    panel_intake_filters, panel_intake_result, save_raw,
    separator_masks, separator_verdict, verdict_label, view_separator,
)
from benefits import (                                      # noqa: E402
    BOX_NAMES, POINTS, DEFAULT_CFG as BENEFITS_DEFAULTS,
    analyze as benefits_analyze, masks_for as benefits_masks,
)

BENEFITS_CONFIG = Path(os.environ.get(
    "LARC_BENEFITS_CONFIG", REPO / "benefits" / "benefits_config.json"))

JPEG_QUALITY = 72
TARGET_STREAM_FPS = 20
SEPARATOR_HOLD_MS = 250
class Store:

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._new = threading.Condition(self._lock)
        self._jpeg: dict[str, bytes] = {}
        self._version = 0
        self.state: dict = {}

    def put(self, name: str, image) -> None:
        ok, buf = cv2.imencode(".jpg", image,
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return
        with self._new:
            self._jpeg[name] = buf.tobytes()
            self._version += 1
            self._new.notify_all()

    def wait_for(self, name: str, seen: int, timeout: float = 2.0):
        """Block until a newer frame exists. Returns (jpeg, version)."""
        with self._new:
            if self._version <= seen:
                self._new.wait(timeout)
            return self._jpeg.get(name), self._version

    def set_state(self, state: dict) -> None:
        with self._lock:
            self.state = state

    def get_state(self) -> dict:
        with self._lock:
            return dict(self.state)


STORE = Store()
LIVE_TEENSY = None

# Config values live here; the HTTP threads write, the worker reads.
CFG_LOCK = threading.Lock()
CALIB = {"intake": None, "separator": None}
CALIB_MODE = "observe"


def calib_status(group, manual):
    harness = CALIB.get(group)
    if harness is None:
        return {"mode": CALIB_MODE, "lines": ["starting"], "deltas": {}}
    return {"mode": harness.mode,
            "lines": harness.lines(manual),
            "deltas": {k: list(v) for k, v in harness.deltas(manual).items()}}
INTAKE_RAW: dict = {}
SEPARATOR_RAW: dict = {}
INTAKE_SPEC: list = []
SEPARATOR_SPEC: list = []
BENEFITS_RAW: dict = {}
BENEFITS_SPEC: list = []


def snapshot_cfg():
    with CFG_LOCK:
        return dict(INTAKE_RAW), dict(SEPARATOR_RAW), dict(BENEFITS_RAW)


BOX_BGR = {0: (150, 150, 150), 1: (40, 40, 230), 2: (230, 120, 30)}


def view_benefits(frame, result, fps):
    out = frame.copy()
    for label, colour, (x1, y1, x2, y2) in zip("LCR", result["colours"],
                                               result["boxes"]):
        cv2.rectangle(out, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), (255, 255, 255), 2)
        cv2.rectangle(out, (x1 - 2, y1 - 24), (x1 + 16, y1 - 6), (0, 0, 0), -1)
        cv2.putText(out, label, (x1, y1 - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    BOX_BGR[colour], 2)
    text = (f"{result['align']}  box={BOX_NAMES[result['box']]}  "
            f"VISION:FE:{result['box']:02X}  {fps:.0f} fps")
    cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (80, 220, 120) if result["align"] == "CENTER" else (220, 220, 220), 2)
    return out


def view_benefits_masks(frame, cfg):
    red, blue = benefits_masks(frame, cfg)
    panel = np.zeros_like(frame)
    panel[red > 0] = BOX_BGR[1]
    panel[blue > 0] = BOX_BGR[2]
    return cv2.resize(panel, (frame.shape[1] // 2, frame.shape[0] // 2))


#  Teensy link (optional)
from beans_bench import BeansBench as TeensyDriver


#  Worker: run both pipelines, encode panels, publish state
def worker(intake, separator, benefits, teensy, rotate,
           stop: threading.Event) -> None:
    from autocalib import DebugHarness
    from intake_autocalib import IntakeCalibrator
    from intake_autocalib import load_seed as load_intake_seed
    from separator_autocalib import SeparatorCalibrator
    from separator_autocalib import load_seed as load_separator_seed

    with CFG_LOCK:
        _i_raw = dict(INTAKE_RAW)
    _i_seed, _i_note = load_intake_seed(derive_intake(_i_raw))
    _s_seed = load_separator_seed()
    CALIB["intake"] = DebugHarness(
        lambda manual: IntakeCalibrator(manual, seed=_i_seed), mode=CALIB_MODE)
    CALIB["separator"] = DebugHarness(
        lambda manual: SeparatorCalibrator(manual, seed=_s_seed), mode=CALIB_MODE)
    print(f"[Calib] intake: {_i_note}")
    print("[Calib] separator: " + ("seed loaded" if _s_seed else
          "no seed file — photometric only, hue locked"))

    counters = dict(left=0, right=0, warm=0, cool=0, centred=0)
    last_align = "NONE"
    last_log = 0.0
    last_frames = {"intake": 0.0, "separator": 0.0}
    last_results = {"intake": None, "separator": None}
    intervals = {"intake": None, "separator": None}

    def timing(role, stamp, started, algorithm_ms):
        now = time.monotonic()
        fresh = stamp != last_frames[role]
        if fresh:
            previous = last_results[role]
            intervals[role] = round((now - previous) * 1000, 1) if previous else None
            last_results[role] = now
            last_frames[role] = stamp
        return dict(decision_ms=round(algorithm_ms, 1),
                    processing_ms=round((now - started) * 1000, 1),
                    frame_age_ms=round((now - stamp) * 1000, 1),
                    output_interval_ms=intervals[role], new_frame=fresh)

    while not stop.is_set():
        intake_raw, separator_raw, benefits_raw = snapshot_cfg()
        intake_hits = None
        verdict = None
        sep_box = None
        intake_timing = separator_timing = None
        intake_stamp = separator_stamp = 0.

        # Intake
        if intake is not None and intake.ok:
            frame, capture_stamp = intake.latest_timed()
            intake_stamp = capture_stamp
            if frame is not None:
                started = time.monotonic()
                intake_manual = derive_intake(intake_raw)
                cfg = CALIB["intake"].update(intake_manual, frame)
                half_w = frame.shape[1] // 2
                filters, results, intake_hits = [], [], {}
                algorithm_ms = (time.monotonic() - started) * 1000
                for side, half in (("left", frame[:, :half_w]),
                                   ("right", frame[:, half_w:])):
                    algorithm_start = time.monotonic()
                    masks = intake_masks(half, cfg)
                    cdata = intake_detect(masks["morph"], cfg, side)
                    hit = intake_trigger(cdata, cfg)
                    intake_hits[side] = hit
                    if hit:
                        counters[side] += 1
                    algorithm_ms += (time.monotonic() - algorithm_start) * 1000
                    filters.append(panel_intake_filters(half, masks, rotate))
                    results.append(panel_intake_result(half, cdata, cfg, side,
                                                       hit, rotate))
                intake_timing = timing("intake", capture_stamp, started, algorithm_ms)
                STORE.put("intake", np.hstack(results))
                STORE.put("intake_filters", np.hstack(filters))

        # Separator
        if separator is not None and separator.ok:
            frame, capture_stamp = separator.latest_timed()
            separator_stamp = capture_stamp
            if frame is not None:
                started = time.monotonic()
                sep_manual = derive_separator(separator_raw)
                rx, ry, rw, rh = clamp_roi(sep_manual, frame.shape[1],
                                           frame.shape[0])
                cfg = CALIB["separator"].update(
                    sep_manual, frame[ry:ry + rh, rx:rx + rw])
                sep_box = dict(x=rx, y=ry, w=rw, h=rh,
                               frame_w=frame.shape[1], frame_h=frame.shape[0])
                roi_hsv = cv2.cvtColor(frame[ry:ry + rh, rx:rx + rw],
                                       cv2.COLOR_BGR2HSV)
                masks = separator_masks(roi_hsv, cfg)
                verdict = separator_verdict(masks, cfg, max(1, rw * rh))
                if verdict["warm"]:
                    counters["warm"] += 1
                if verdict["cool"]:
                    counters["cool"] += 1
                algorithm_ms = (time.monotonic() - started) * 1000
                rendered = view_separator(frame, cfg, (rx, ry, rw, rh),
                                          verdict, masks, separator.fps, True)
                separator_timing = timing("separator", capture_stamp, started, algorithm_ms)
                STORE.put("separator", rendered)

        # Benefits
        ben = None
        if benefits is not None and benefits.ok:
            frame = benefits.latest()
            if frame is not None:
                ben = benefits_analyze(frame, benefits_raw)
                if ben["align"] == "CENTER" and last_align != "CENTER":
                    counters["centred"] += 1
                last_align = ben["align"]
                STORE.put("benefits", view_benefits(frame, ben, benefits.fps))
                STORE.put("benefits_filters",
                          view_benefits_masks(frame, benefits_raw))
                ben["frame_w"], ben["frame_h"] = frame.shape[1], frame.shape[0]

        # Teensy
        teensy_state = None
        if teensy is not None:
            try:
                teensy.push(intake_hits, verdict, intake_stamp, separator_stamp)
                teensy_state = teensy.snapshot()
            except Exception as exc:                        # noqa: BLE001
                teensy_state = dict(device=teensy.device, error=str(exc),
                                    healthy=False)

        # Publish
        payload_intake = None
        if intake_hits is not None:
            payload_intake = (int(intake_hits["right"]) |
                              (int(intake_hits["left"]) << 1))

        STORE.set_state(dict(
            intake=dict(
                ok=bool(intake is not None and intake.ok),
                error=(intake.error if intake is not None else "disabled"),
                fps=round(intake.fps, 1) if intake is not None and intake.ok else 0,
                left=bool(intake_hits and intake_hits["left"]),
                right=bool(intake_hits and intake_hits["right"]),
                vision=(f"VISION:{payload_intake:02X}"
                        if payload_intake is not None else "-"),
                timing=intake_timing,
                values=intake_raw, spec=INTAKE_SPEC,
                config=str(INTAKE_CONFIG),
                calib=calib_status("intake", derive_intake(intake_raw)),
            ),
            separator=dict(
                ok=bool(separator is not None and separator.ok),
                error=(separator.error if separator is not None else "disabled"),
                fps=round(separator.fps, 1) if separator is not None and separator.ok else 0,
                label=verdict_label(verdict)[0] if verdict else "-",
                warm_pct=round(verdict["warm_pct"], 1) if verdict else 0,
                cool_pct=round(verdict["cool_pct"], 1) if verdict else 0,
                green_pct=round(verdict["green_pct"], 1) if verdict else 0,
                white_pct=round(verdict["white_pct"], 1) if verdict else 0,
                reason=verdict.get("reason", "-") if verdict else "-",
                vision=(f"VISION:FD:{int(verdict['warm']):02X}:"
                        f"{int(verdict['cool']):02X}") if verdict else "-",
                roi=sep_box,
                timing=separator_timing,
                values=separator_raw, spec=SEPARATOR_SPEC,
                config=str(SEPARATOR_CONFIG),
                calib=calib_status("separator",
                                   derive_separator(separator_raw)),
            ),
            benefits=dict(
                ok=bool(benefits is not None and benefits.ok),
                error=(benefits.error if benefits is not None else "disabled"),
                fps=round(benefits.fps, 1) if benefits is not None and benefits.ok else 0,
                align=ben["align"] if ben else "-",
                box=BOX_NAMES[ben["box"]] if ben else "-",
                rois=[BOX_NAMES[c] for c in ben["colours"]] if ben else [],
                pcts=[[round(r), round(b)] for r, b in ben["pcts"]] if ben else [],
                vision=f"VISION:FE:{ben['box']:02X}" if ben else "-",
                frame_w=ben["frame_w"] if ben else 640,
                frame_h=ben["frame_h"] if ben else 480,
                values=benefits_raw, spec=BENEFITS_SPEC,
                config=str(BENEFITS_CONFIG),
            ),
            teensy=teensy_state,
            counters=counters,
        ))

        now = time.monotonic()
        if now - last_log >= 5.0:
            last_log = now
            state = STORE.get_state()
            print(f"  intake {state['intake']['vision']:<12} "
                  f"{state['intake']['fps']:>5} fps   |   "
                  f"sep {state['separator']['label']:<24} "
                  f"{state['separator']['vision']:<18} "
                  f"{state['separator']['fps']:>5} fps", flush=True)
            for role in ("intake", "separator"):
                measured = state[role].get("timing")
                if measured:
                    print(f"[opencv-{role}] decision={measured['decision_ms']}ms "
                          f"processing={measured['processing_ms']}ms "
                          f"frame_age={measured['frame_age_ms']}ms "
                          f"interval={measured['output_interval_ms']}ms "
                          f"new_frame={measured['new_frame']}", flush=True)

        # Both cameras top out at 30 fps; there is nothing to gain from
        # spinning faster than the slower of them delivers.
        time.sleep(0.01)


#  The page
PAGE = r"""<!doctype html>
<meta charset="utf-8">
<title>BEANS / BENEFITS tuner</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#111417; color:#e6e6e6;
         font:13px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace; }
  header { padding:10px 16px; background:#181c20; border-bottom:1px solid #262c33;
           display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
  h1 { font-size:14px; margin:0; font-weight:600; letter-spacing:.02em; }
  button { background:#232a31; color:#e6e6e6; border:1px solid #39424c;
           border-radius:5px; padding:5px 11px; cursor:pointer; font:inherit; }
  button:hover { background:#2d363f; }
  button.go { background:#1f4d33; border-color:#2e6b48; }
  main { display:flex; gap:14px; padding:14px; align-items:flex-start;
         flex-wrap:wrap; }
  .col { flex:1 1 460px; min-width:340px; }
  .card { background:#161a1e; border:1px solid #262c33; border-radius:8px;
          margin-bottom:14px; overflow:hidden; }
  .card h2 { font-size:12px; margin:0; padding:8px 12px; background:#1b2026;
             border-bottom:1px solid #262c33; text-transform:uppercase;
             letter-spacing:.06em; color:#9fb0c0; }
  .card .body { padding:10px 12px; }
  img { display:block; width:100%; height:auto; background:#000; }
  .stat { display:flex; gap:14px; flex-wrap:wrap; padding:8px 12px;
          border-bottom:1px solid #262c33; }
  .stat b { color:#7fd6a0; font-weight:600; }
  .bad { color:#ff8a7a; }
  .warn { color:#ffbf5a; }
  .row { display:grid; grid-template-columns:104px 1fr 52px; gap:8px;
         align-items:center; margin:3px 0; }
  .row label { color:#93a3b3; font-size:11px; }
  .row input[type=range] { width:100%; }
  .row output { text-align:right; color:#dfe7ee; }
  #sep-wrap { position:relative; cursor:crosshair; }
  #sep-box { position:absolute; border:2px dashed #7fd6a0; display:none;
             pointer-events:none; }
  .hint { color:#7c8b99; padding:0 12px 10px; font-size:11px; }
  #ben-img { cursor:crosshair; }
  button.sel { background:#1f4d33; border-color:#2e6b48; }
</style>

<header>
  <h1>BEANS / BENEFITS tuner</h1>
  <button class=go onclick="save('both')">Save all</button>
  <button onclick="save('intake')">Save intake</button>
  <button onclick="save('separator')">Save separator</button>
  <button onclick="save('benefits')">Save benefits</button>
  <button onclick="post('/reload').then(load)">Reload from disk</button>
  <button onclick="printJson()">Print JSON</button>
  <button id=calibBtn onclick="post('/calib').then(load)">autocalib: ?</button>
  <span id=calibInfo class=muted></span>
  <span id=msg style="color:#7fd6a0"></span><pre id=teensy-feedback></pre>
</header>
<p class=hint>Timing: decision = detection and ROI logic; processing also includes preview drawing;
frame age = OpenCV delivery to result. New-result interval excludes repeated processing of the same frame.
These do not include sensor buffering, browser delivery, Teensy or servo movement.</p>

<main>
  <div class=col>
    <div class=card>
      <h2>Intake &mdash; ZED</h2>
      <div class=stat id=intake-stat></div>
      <img draggable="false" data-preview="intake">
    </div>
    <div class=card>
      <h2>Intake filters</h2>
      <div class=hint>orange = saturated &nbsp; blue = dark &nbsp; white = both
        &nbsp; red = background green removed &nbsp; yellow = final edge</div>
      <img draggable="false" data-preview="intake_filters">
    </div>
    <div class=card>
      <h2>orin_config.json</h2>
      <div class=body id=intake-sliders></div>
    </div>
  </div>

  <div class=col>
    <div class=card>
      <h2>Separator &mdash; C920</h2>
      <div class=stat id=separator-stat></div>
      <div id=sep-wrap>
        <img id=sep-img draggable="false" data-preview="separator">
        <div id=sep-box></div>
      </div>
      <div class=hint>Drag a box on the image to set the ROI, then Save separator.</div>
    </div>
    <div class=card>
      <h2>separator_config.json</h2>
      <div class=body id=separator-sliders></div>
    </div>
  </div>

  <div class=col>
    <div class=card>
      <h2>Benefits &mdash; C920</h2>
      <div class=stat id=benefits-stat></div>
      <img id=ben-img draggable="false" data-preview="benefits">
      <div class=hint>Pick a point, then click the image to place it. Box is
        reported only when L, C and R see the same colour.
        <button id=pt-p1 class=sel onclick="pickPoint('p1')">L</button>
        <button id=pt-p2 onclick="pickPoint('p2')">C</button>
        <button id=pt-p3 onclick="pickPoint('p3')">R</button></div>
    </div>
    <div class=card>
      <h2>Benefits masks</h2>
      <div class=hint>red = red mask &nbsp; blue = blue mask</div>
      <img draggable="false" data-preview="benefits_filters">
    </div>
    <div class=card>
      <h2>benefits_config.json</h2>
      <div class=body id=benefits-sliders></div>
    </div>
  </div>
</main>

<script>
const builtSpecs = {};
// Finite image requests keep HTTP connections available for controls.
for (const img of document.querySelectorAll('[data-preview]')) {
  const refresh = () => { img.src = '/snapshot/' + img.dataset.preview + '?t=' + Date.now(); };
  img.addEventListener('load', () => setTimeout(refresh, 150));
  img.addEventListener('error', () => setTimeout(refresh, 1000));
  img.addEventListener('dragstart', ev => ev.preventDefault());
  refresh();
}

function post(url) { return fetch(url, {method:'POST'}); }

function flash(text, bad) {
  const m = document.getElementById('msg');
  m.textContent = text;
  m.style.color = bad ? '#ff8a7a' : '#7fd6a0';
  setTimeout(() => { m.textContent = ''; }, 2500);
}

function save(which) {
  post('/save?group=' + which)
    .then(r => r.json())
    .then(j => flash(j.message, !j.ok));
}

function printJson() {
  fetch('/state').then(r => r.json()).then(j => {
    console.log('orin_config.json', j.intake.values);
    console.log('separator_config.json', j.separator.values);
    console.log('benefits_config.json', j.benefits.values);
    flash('printed to the browser console');
  });
}

function timingLine(t) {
  if (!t) return '<span>timing: waiting for frame</span>';
  return `<span>decision <b>${t.decision_ms} ms</b></span>
    <span>processing ${t.processing_ms} ms</span>
    <span>frame age ${t.frame_age_ms} ms</span>
    <span>new-result interval ${t.output_interval_ms ?? '—'} ms</span>
    <span>${t.new_frame ? 'fresh frame' : 'reprocessed frame'}</span>`;
}

function buildSliders(group, spec, values) {
  const host = document.getElementById(group + '-sliders');
  host.innerHTML = '';
  for (const [key, max] of spec) {
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML = `<label for="${group}-${key}">${key}</label>
      <input type=range id="${group}-${key}" min=0 max="${max}"
             value="${values[key]}">
      <output id="${group}-${key}-out">${values[key]}</output>`;
    host.appendChild(row);
    const input = row.querySelector('input');
    input.addEventListener('input', () => {
      document.getElementById(`${group}-${key}-out`).textContent = input.value;
      fetch(`/set?group=${group}&key=${key}&value=${input.value}`,
            {method:'POST'});
    });
  }
}

function syncSliders(group, values) {
  for (const key in values) {
    const input = document.getElementById(`${group}-${key}`);
    if (input && document.activeElement !== input &&
        input.value != values[key]) {
      input.value = values[key];
      document.getElementById(`${group}-${key}-out`).textContent = values[key];
    }
  }
}

function statLine(el, html) { document.getElementById(el).innerHTML = html; }

function load() {
  fetch('/state').then(r => r.json()).then(j => {
    if (!j.intake || !j.separator || !j.benefits) {
      throw new Error('Waiting for camera processing');
    }
    for (const group of ['intake', 'separator', 'benefits']) {
      const spec = j[group].spec || [];
      const signature = JSON.stringify(spec);
      if (builtSpecs[group] !== signature) {
        buildSliders(group, spec, j[group].values);
        builtSpecs[group] = signature;
      } else {
        syncSliders(group, j[group].values);
      }
    }

    // Autocalibration readout — every poll, not just the first.
    const cal = (j.separator.calib || j.intake.calib || {});
    const btn = document.getElementById('calibBtn');
    if (btn && cal.mode) btn.textContent = 'autocalib: ' + cal.mode;
    const info = document.getElementById('calibInfo');
    if (info) {
      const li = (j.intake.calib || {}).lines || [];
      const ls = (j.separator.calib || {}).lines || [];
      info.textContent = [li[1] ? 'intake ' + li[1] : '',
                          ls[1] ? 'sep ' + ls[1] : ''].filter(Boolean).join('   |   ');
    }

    const i = j.intake;
    statLine('intake-stat', i.ok
      ? `<span>L <b>${i.left ? 'HIT' : '&mdash;'}</b></span>
         <span>R <b>${i.right ? 'HIT' : '&mdash;'}</b></span>
         <span>${i.vision}</span><span>${i.fps} fps</span>${timingLine(i.timing)}`
      : `<span class=bad>unavailable &mdash; ${i.error}</span>`);

    const s = j.separator;
    statLine('separator-stat', s.ok
      ? `<span><b>${s.label}</b></span><span>${s.vision}</span>
         <span>warm ${s.warm_pct}</span><span>cool ${s.cool_pct}</span>
         <span>green ${s.green_pct}</span><span>white ${s.white_pct}</span>
         <span>${s.fps} fps</span>${timingLine(s.timing)}`
      : `<span class=bad>unavailable &mdash; ${s.error}</span>`);

    const b = j.benefits;
    statLine('benefits-stat', b.ok
      ? `<span><b>${b.align}</b></span><span>box ${b.box}</span><span>${b.vision}</span>
         <span>L/C/R ${b.rois.join(' / ')}</span>
         <span>r/b% ${b.pcts.map(p => p.join('/')).join('  ')}</span>
         <span>${b.fps} fps</span><span>centred ${j.counters.centred}</span>`
      : `<span class=bad>unavailable &mdash; ${b.error}</span>`);
    window._ben = b;

    if (j.teensy) {
      const t = j.teensy;
      document.getElementById('teensy-feedback').textContent = (t.active ? 'BEANS active' : 'Waiting for BEANS request') + '\n' + (t.messages || []).join('\n');
      document.getElementById('msg').textContent = t.healthy
        ? `teensy ${t.device}  up=${t.upper?1:0} lo=${t.lower?1:0} sep=${t.separator}`
        : `teensy DOWN ${t.error || ''}`;
      document.getElementById('msg').style.color = t.healthy ? '#7fd6a0' : '#ff8a7a';
    }
    window._roi = s.roi;
  }).catch(err => flash('Tuner: ' + err.message, true))
    .finally(() => setTimeout(load, 400));
}

// drag a new ROI on the separator image
(function () {
  const wrap = document.getElementById('sep-wrap');
  const img = document.getElementById('sep-img');
  const box = document.getElementById('sep-box');
  let start = null;

  // The separator panel is the frame with a 140px mask strip stacked under
  // it, so only the top portion maps to frame coordinates.
  function toFrame(ev) {
    const r = img.getBoundingClientRect();
    const roi = window._roi;
    if (!roi) return null;
    const shown_h = r.height * (r.width / r.width);
    const scale = roi.frame_w / r.width;
    return {
      x: Math.round((ev.clientX - r.left) * scale),
      y: Math.round((ev.clientY - r.top) * scale),
      max_y: roi.frame_h
    };
  }

  wrap.addEventListener('mousedown', ev => {
    start = {cx: ev.clientX, cy: ev.clientY, f: toFrame(ev)};
    box.style.display = 'block';
    const r = wrap.getBoundingClientRect();
    box.style.left = (ev.clientX - r.left) + 'px';
    box.style.top = (ev.clientY - r.top) + 'px';
    box.style.width = box.style.height = '0px';
    ev.preventDefault();
  });

  window.addEventListener('mousemove', ev => {
    if (!start) return;
    const r = wrap.getBoundingClientRect();
    const x = Math.min(start.cx, ev.clientX) - r.left;
    const y = Math.min(start.cy, ev.clientY) - r.top;
    box.style.left = x + 'px';
    box.style.top = y + 'px';
    box.style.width = Math.abs(ev.clientX - start.cx) + 'px';
    box.style.height = Math.abs(ev.clientY - start.cy) + 'px';
  });

  window.addEventListener('mouseup', ev => {
    if (!start) return;
    box.style.display = 'none';
    const a = start.f, b = toFrame(ev);
    start = null;
    if (!a || !b) return;
    const x = Math.max(0, Math.min(a.x, b.x));
    const y = Math.max(0, Math.min(a.y, b.y));
    const w = Math.abs(b.x - a.x), h = Math.abs(b.y - a.y);
    if (w < 8 || h < 8 || y > a.max_y) return;
    fetch(`/roi?x=${x}&y=${y}&w=${w}&h=${h}`, {method:'POST'})
      .then(() => flash(`ROI ${w}x${h} @ (${x},${y}) — Save separator to keep it`));
  });
})();

// click to place the selected benefits ROI
let point = 'p1';
function pickPoint(p) {
  point = p;
  for (const q of ['p1', 'p2', 'p3'])
    document.getElementById('pt-' + q).className = q === p ? 'sel' : '';
}
document.getElementById('ben-img').addEventListener('click', ev => {
  const b = window._ben;
  if (!b || !b.ok) return flash('Benefits camera is not ready', true);
  const r = ev.target.getBoundingClientRect();
  const x = Math.round((ev.clientX - r.left) * b.frame_w / r.width);
  const y = Math.round((ev.clientY - r.top) * b.frame_h / r.height);
  const selected = point;
  fetch(`/point?p=${selected}&x=${x}&y=${y}`, {method:'POST'})
    .then(r => r.json()).then(j => {
      if (!j.ok) throw new Error(j.message || 'Point rejected');
      flash(`${selected} @ (${x},${y}) — Save benefits to keep it`);
    }).catch(err => flash(err.message, true));
});

load();

</script>
"""


#  HTTP
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    # helpers
    def _send(self, code, body: bytes, ctype="text/plain; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload, code=200):
        self._send(code, json.dumps(payload).encode(), "application/json")

    # -- routes --
    def do_GET(self):
        route = urlparse(self.path)
        path, query = route.path, parse_qs(route.query)

        if path == "/":
            return self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        if path == "/state":
            state = STORE.get_state()
            if LIVE_TEENSY is not None:
                state['teensy'] = LIVE_TEENSY.snapshot()
            return self._json(state)
        if path.startswith("/snapshot/"):
            with STORE._lock:
                jpeg = STORE._jpeg.get(path.rsplit("/", 1)[-1])
            if jpeg is None:
                return self._send(503, b"waiting for camera")
            return self._send(200, jpeg, "image/jpeg")
        if path.startswith("/stream/"):
            return self._stream(path.rsplit("/", 1)[-1])
        if path in ("/set", "/save", "/reload", "/roi", "/calib", "/point"):
            return self._mutate(path, query)
        return self._send(404, b"not found")

    def do_POST(self):
        route = urlparse(self.path)
        return self._mutate(route.path, parse_qs(route.query))

    def _mutate(self, path, query):
        global INTAKE_RAW, SEPARATOR_RAW, BENEFITS_RAW

        def one(name, default=None):
            return query.get(name, [default])[0]

        if path == "/set":
            group, key, value = one("group"), one("key"), one("value")
            spec = {"intake": INTAKE_SPEC, "benefits": BENEFITS_SPEC}.get(
                group, SEPARATOR_SPEC)
            allowed = {k: hi for k, hi in spec}
            if key not in allowed:
                return self._json({"ok": False, "message": f"unknown key {key}"}, 400)
            try:
                clamped = max(0, min(int(value), allowed[key]))
            except (TypeError, ValueError):
                return self._json({"ok": False, "message": "bad value"}, 400)
            with CFG_LOCK:
                target = {"intake": INTAKE_RAW, "benefits": BENEFITS_RAW}.get(
                    group, SEPARATOR_RAW)
                target[key] = clamped
            return self._json({"ok": True, "value": clamped})

        if path == "/calib":
            global CALIB_MODE
            want = one("mode")
            for harness in CALIB.values():
                if harness is None:
                    continue
                if want in harness.MODES:
                    harness.mode = want
                    if want == "off":
                        harness.calib = None
                        harness._seed = None
                else:
                    harness.cycle()
            live = next((h for h in CALIB.values() if h is not None), None)
            CALIB_MODE = live.mode if live else CALIB_MODE
            print(f"[calib] mode -> {CALIB_MODE.upper()}")
            return self._json({"ok": True, "mode": CALIB_MODE})

        if path == "/roi":
            try:
                box = {k: int(one(k)) for k in ("x", "y", "w", "h")}
            except (TypeError, ValueError):
                return self._json({"ok": False, "message": "bad roi"}, 400)
            with CFG_LOCK:
                SEPARATOR_RAW.update(roi_x=box["x"], roi_y=box["y"],
                                     roi_w=box["w"], roi_h=box["h"])
            print(f"[roi] {box['w']}x{box['h']} @ ({box['x']},{box['y']})"
                  f"  — not saved yet")
            return self._json({"ok": True})

        if path == "/point":
            name = one("p")
            try:
                x, y = int(one("x")), int(one("y"))
            except (TypeError, ValueError):
                return self._json({"ok": False, "message": "bad point"}, 400)
            if name not in POINTS:
                return self._json({"ok": False, "message": f"unknown point {name}"}, 400)
            with CFG_LOCK:
                BENEFITS_RAW.update({f"{name}_x": x, f"{name}_y": y})
            print(f"[point] {name} @ ({x},{y})  — not saved yet")
            return self._json({"ok": True})

        if path == "/save":
            group = one("group", "both")
            intake_raw, separator_raw, benefits_raw = snapshot_cfg()
            written = []
            try:
                if group in ("intake", "both") and intake_raw:
                    save_raw(INTAKE_CONFIG, intake_raw)
                    written.append(INTAKE_CONFIG.name)
                if group in ("separator", "both") and separator_raw:
                    save_raw(SEPARATOR_CONFIG, separator_raw)
                    written.append(SEPARATOR_CONFIG.name)
                if group in ("benefits", "both") and benefits_raw:
                    save_raw(BENEFITS_CONFIG, benefits_raw)
                    written.append(BENEFITS_CONFIG.name)
            except OSError as exc:
                return self._json({"ok": False, "message": f"save failed: {exc}"}, 500)
            return self._json({"ok": True,
                               "message": "saved " + ", ".join(written)
                               if written else "nothing to save"})

        if path == "/reload":
            with CFG_LOCK:
                INTAKE_RAW.update(load_raw(INTAKE_CONFIG, INTAKE_DEFAULTS))
                SEPARATOR_RAW.update(load_raw(SEPARATOR_CONFIG, SEPARATOR_DEFAULTS))
                BENEFITS_RAW.update(load_raw(BENEFITS_CONFIG, BENEFITS_DEFAULTS))
            return self._json({"ok": True, "message": "reloaded"})

        return self._send(404, b"not found")

    def _stream(self, name):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seen = 0
        period = 1.0 / TARGET_STREAM_FPS
        try:
            while True:
                jpeg, seen = STORE.wait_for(name, seen)
                if jpeg is None:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpeg)).encode() +
                                 b"\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass


#  Main
def build_specs(intake, separator, benefits=None):
    global INTAKE_SPEC, SEPARATOR_SPEC, BENEFITS_SPEC

    if benefits is not None and benefits.ok:
        w, h = benefits.width, benefits.height
        BENEFITS_SPEC = [
            ["p1_x", w], ["p1_y", h], ["p2_x", w], ["p2_y", h],
            ["p3_x", w], ["p3_y", h], ["roi_size", 80],
            ["red_h_lo1", 180], ["red_h_hi1", 180],
            ["red_h_lo2", 180], ["red_h_hi2", 180],
            ["red_s_min", 255], ["red_v_min", 255],
            ["blue_h_lo", 180], ["blue_h_hi", 180],
            ["blue_s_min", 255], ["blue_v_min", 255],
            ["min_pct", 100], ["morph_k", 15], ["morph_iter", 4],
        ]

    if intake is not None and intake.ok:
        half_w, h = intake.width // 2, intake.height
        INTAKE_SPEC = [
            ["s_min", 255], ["v_dark", 255],
            ["bg_h_lo", 179], ["bg_h_hi", 179],
            ["bg_s_min", 255], ["bg_v_min", 255],
            ["area_min", 4000], ["area_max", 30000],
            ["rad_min", 300], ["rad_max", 400],
            ["circ_min", 100], ["circ_max", 100],
            ["morph_k", 21], ["morph_iter", 6],
            ["det_cy_l", max(1, half_w)], ["det_cy_r", max(1, half_w)],
            ["det_thick", max(1, half_w)],
            ["trigger_x", max(1, h)], ["trig_y2", max(1, h)],
        ]

    if separator is not None and separator.ok:
        w, h = separator.width, separator.height
        with CFG_LOCK:
            if (SEPARATOR_RAW["roi_x"] + SEPARATOR_RAW["roi_w"] > w or
                    SEPARATOR_RAW["roi_y"] + SEPARATOR_RAW["roi_h"] > h):
                print(f"[WARN] separator ROI {SEPARATOR_RAW['roi_w']}x"
                      f"{SEPARATOR_RAW['roi_h']} @ ({SEPARATOR_RAW['roi_x']},"
                      f"{SEPARATOR_RAW['roi_y']}) does not fit this {w}x{h} frame.")
                print(f"[WARN] separator_vision.py clamps it to "
                      f"{clamp_roi(SEPARATOR_RAW, w, h)} — a few pixels in the "
                      f"corner. Drag a new box on the page and save.")
                SEPARATOR_RAW["roi_x"] = min(SEPARATOR_RAW["roi_x"], w // 4)
                SEPARATOR_RAW["roi_y"] = min(SEPARATOR_RAW["roi_y"], h // 4)
                SEPARATOR_RAW["roi_w"] = min(SEPARATOR_RAW["roi_w"], w // 2)
                SEPARATOR_RAW["roi_h"] = min(SEPARATOR_RAW["roi_h"], h // 2)
        SEPARATOR_SPEC = [
            ["roi_x", max(1, w - 1)], ["roi_y", max(1, h - 1)],
            ["roi_w", w], ["roi_h", h],
            ["warm_h_lo", 179], ["warm_h_hi", 179],
            ["warm_s_min", 255], ["warm_v_min", 255],
            ["cool_h_lo", 179], ["cool_h_hi", 179],
            ["cool_s_min", 255], ["cool_v_min", 255],
            ["green_h_lo", 179], ["green_h_hi", 179],
            ["green_s_min", 255], ["green_v_min", 255],
            ["black_v_max", 255],
            ["white_s_max", 255], ["white_v_min", 255],
            ["warm_frac", 100], ["cool_frac", 100], ["green_frac", 100],
            ["white_frac", 100], ["unknown_to_mature", 1],
            ["unknown_cover_min", 100],
            ["morph_k", 21], ["morph_iter", 6],
        ]


def main() -> int:
    global INTAKE_RAW, SEPARATOR_RAW, BENEFITS_RAW, LIVE_TEENSY

    parser = argparse.ArgumentParser(
        description="Browser-based tuning for both beans-phase cameras. "
                    "No GUI OpenCV required.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="127.0.0.1",
                        help="0.0.0.0 to reach it from other machines "
                             "(default: localhost only)")
    parser.add_argument("--teensy", action="store_true",
                        help="also stream real BEANS frames to the Teensy — "
                             "stop the dispatcher first")
    parser.add_argument("--serial-device", help="Explicit Teensy serial port")
    parser.add_argument("--intake-device", help="open by index/path instead of role")
    parser.add_argument("--separator-device", help="same, for the separator")
    parser.add_argument("--no-intake", action="store_true")
    parser.add_argument("--no-separator", action="store_true")
    parser.add_argument("--benefits-device", help="same, for the benefits camera")
    parser.add_argument("--no-benefits", action="store_true")
    parser.add_argument("--no-rotate", action="store_true",
                        help="do not rotate the intake panels 90 deg")
    args = parser.parse_args()

    INTAKE_RAW = load_raw(INTAKE_CONFIG, INTAKE_DEFAULTS)
    SEPARATOR_RAW = load_raw(SEPARATOR_CONFIG, SEPARATOR_DEFAULTS)
    BENEFITS_RAW = load_raw(BENEFITS_CONFIG, BENEFITS_DEFAULTS)

    print("[cam] opening — 'Device or resource busy' means the dispatcher "
          "still owns them:  sudo systemctl stop larc-dispatcher")
    intake = None if args.no_intake else CameraThread("intake", device=args.intake_device)
    separator = None if args.no_separator else CameraThread(
        "separator", device=args.separator_device)
    benefits = None if args.no_benefits else CameraThread(
        "benefits", device=args.benefits_device)
    cams = (intake, separator, benefits)

    for cam in cams:
        if cam is None:
            continue
        if cam.ok:
            print(f"[cam] {cam.role:<10} {cam.info.device}  "
                  f"{cam.width}x{cam.height}  ({cam.info.describe()})")
        else:
            print(f"[cam] {cam.role:<10} UNAVAILABLE — {cam.error}")

    if not any(c is not None and c.ok for c in cams):
        print("[fatal] no camera opened. Nothing to serve.")
        return 1

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if all(c.latest() is not None
               for c in cams if c is not None and c.ok):
            break
        time.sleep(0.05)

    build_specs(intake, separator, benefits)

    teensy = None
    if args.teensy:
        try:
            teensy = TeensyDriver(device=args.serial_device)
            LIVE_TEENSY = teensy
        except Exception as exc: # noqa: BLE001
            print(f"[teensy] NOT started — {exc}")
            for cam in cams:
                if cam is not None: cam.release()
            return 1

    stop = threading.Event()
    thread = threading.Thread(target=worker,
                              args=(intake, separator, benefits, teensy,
                                    not args.no_rotate, stop),
                              name="pipeline", daemon=True)
    thread.start()

    try:
        server = ThreadingHTTPServer((args.bind, args.port), Handler)
    except BaseException:
        stop.set()
        if teensy is not None: teensy.close()
        for cam in cams:
            if cam is not None: cam.release()
        raise
    server.daemon_threads = True

    host = socket.gethostname()
    print(f"\n  open  http://localhost:{args.port}   "
          f"(VS Code forwards this over your SSH session)")
    if args.bind != "127.0.0.1":
        print(f"  also  http://{host}:{args.port}  — reachable on the network")
    print(f"  configs: {INTAKE_CONFIG}\n           {SEPARATOR_CONFIG}"
          f"\n           {BENEFITS_CONFIG}")
    print("  Ctrl+C to stop\n")

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[stopping]")
    finally:
        stop.set()
        server.server_close()
        if teensy is not None:
            teensy.close()
        for cam in cams:
            if cam is not None:
                cam.release()
        state = STORE.get_state()
        counters = state.get("counters", {})
        if counters:
            print(f"[stopped] intake L={counters.get('left')} "
                  f"R={counters.get('right')}   separator "
                  f"warm={counters.get('warm')} cool={counters.get('cool')}   "
                  f"benefits centred={counters.get('centred')}")
        print("[reminder] sudo systemctl start larc-dispatcher")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
