#!/usr/bin/env python3
"""Dashboard en vivo tipo Serial Plotter (Arduino) para la camara del
SORTER/separador: UNA sola ventana de matplotlib, 1 fila x 3 columnas --
RGB (promedio del ROI) | camara con el ROI y LOS DOS veredictos | HSV
(promedio del ROI). Mismo lenguaje visual que dashboard-xpgm.py.

Clasificacion: usa sorter-xpgm.py, una copia AISLADA de la logica HSV real
de separator_vision.py (no la importa en vivo -- si ese pipeline cambia,
este dashboard no se entera, y viceversa), mas un clasificador RGB nuevo
(mismos centroides que RGB-xpgm.py) para comparar en vivo si vale la pena
cambiar de HSV a RGB. El panel de camara muestra los dos veredictos a la vez.

Camara: se abre por ROL ("separator") via vision_orin/link/camera_select.py
-- eso SI se reusa en vivo, a proposito: es infraestructura de identidad de
USB (que /dev/videoN te toca hoy), no logica de clasificacion de color, y
reimplementarla aqui solo agregaria un bug nuevo sin servir de experimento.
El ROI y los umbrales HSV se leen de separator_config.json real (datos, no
codigo) con fallback a DEFAULT_CFG de sorter-xpgm.py si no existe.

Requiere escritorio (igual que detector-xpgm.py --preview): correr desde el
escritorio remoto, no por SSH puro.
"""
import argparse
import importlib.util
import json
import statistics
import sys
from collections import deque
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
SEPARATOR_DIR = HERE.parent / 'separator'
LINK_DIR = HERE.parent / 'link'


def median_last_n(dq, n):
    """x(t) = median(x(t-1), ..., x(t-n)) -- sugerencia del Dr. Bortoni para
    no decidir con un solo cuadro ruidoso. n grande = mas estable pero mas
    lento para reaccionar; n chico = reacciona rapido pero deja pasar mas
    ruido. Con los ultimos < n cuadros disponibles (arranque), usa los que
    haya en vez de fallar."""
    window = list(dq)[-n:]
    return statistics.median(window) if window else float('nan')


def _median_series(dq, n):
    """Mediana movil sobre TODO el buffer, para dibujar la linea de mediana
    completa en el plot (no solo el ultimo punto). NaN mientras no hay al
    menos n muestras todavia -- matplotlib deja ese tramo como hueco."""
    vals = list(dq)
    out = []
    for i in range(len(vals)):
        window = vals[max(0, i - n + 1):i + 1]
        out.append(statistics.median(window) if len(window) >= n else float('nan'))
    return out


def load_sorter_xpgm():
    spec = importlib.util.spec_from_file_location("sorter_xpgm", HERE / "sorter-xpgm.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def load_cfg(sx):
    """Lee separator_config.json real como DATOS (merge sobre el DEFAULT_CFG
    propio de sorter-xpgm.py) -- no es logica de pipeline, es calibracion ya
    hecha que no tiene sentido reinventar."""
    cfg = dict(sx.DEFAULT_CFG)
    path = SEPARATOR_DIR / "separator_config.json"
    if path.exists():
        saved = json.loads(path.read_text())
        if "roi" in saved:
            r = saved["roi"]
            saved["roi_x"], saved["roi_y"] = r["x"], r["y"]
            saved["roi_w"], saved["roi_h"] = r["w"], r["h"]
        cfg.update(saved)
    return cfg


def open_separator_camera():
    """Unico punto donde este dashboard SI depende de codigo externo a
    xpgm-testVision: la resolucion de camara por identidad USB (no logica de
    color). Ver docstring del modulo."""
    sys.path.insert(0, str(LINK_DIR))
    from camera_select import open_role, CameraNotFound
    try:
        return open_role("separator")
    except CameraNotFound as exc:
        raise SystemExit(f"[ERROR] camara separator: {exc}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--window', type=int, default=150, help='Cuadros visibles en la ventana deslizante')
    p.add_argument('--interval-ms', type=int, default=150, help='Intervalo de refresco del plot (ms)')
    p.add_argument('--median-n', type=int, default=5,
                    help='Ventana del filtro de mediana que decide el veredicto (sugerencia Dr. Bortoni)')
    args = p.parse_args()

    sx = load_sorter_xpgm()
    cfg = load_cfg(sx)
    cap, cam = open_separator_camera()

    rx, ry, rw, rh = cfg['roi_x'], cfg['roi_y'], cfg['roi_w'], cfg['roi_h']
    roi_pixels = rw * rh

    N = args.window
    t = deque(maxlen=N)
    buf = {ch: deque(maxlen=N) for ch in ('R', 'G', 'B', 'H', 'S', 'V')}
    frame_idx = [0]

    chan_color = {'R': '#d03b3b', 'G': '#0ca30c', 'B': '#2a78d6',
                  'H': '#9b59b6', 'S': '#e67e22', 'V': '#7f8c8d'}

    fig, (ax_rgb, ax_cam, ax_hsv) = plt.subplots(1, 3, figsize=(14, 5), gridspec_kw={'width_ratios': [1.1, 0.9, 1.1]})
    fig.canvas.manager.set_window_title('Dashboard sorter en vivo (xpgm) - RGB | camara | HSV')

    # Cruda = delgada y tenue (el dato tal cual); mediana = gruesa y solida
    # (lo que de verdad decide el veredicto). Mismo color por canal para que
    # se lean como el mismo canal, dos vistas.
    lines_rgb, lines_rgb_med, lines_hsv, lines_hsv_med = {}, {}, {}, {}
    for ch in ('R', 'G', 'B'):
        (lines_rgb[ch],) = ax_rgb.plot([], [], '-', color=chan_color[ch], alpha=0.35, linewidth=1.1)
        (lines_rgb_med[ch],) = ax_rgb.plot([], [], '-', color=chan_color[ch], label=f'{ch} (mediana)', linewidth=1.8)
    for ch in ('H', 'S', 'V'):
        (lines_hsv[ch],) = ax_hsv.plot([], [], '-', color=chan_color[ch], alpha=0.35, linewidth=1.1)
        (lines_hsv_med[ch],) = ax_hsv.plot([], [], '-', color=chan_color[ch], label=f'{ch} (mediana)', linewidth=1.8)

    ax_rgb.set_ylim(0, 255)
    ax_rgb.set_xlabel('cuadro')
    ax_rgb.set_ylabel('canal (0-255), promedio del ROI')
    ax_rgb.set_title('RGB - ROI sorter (tenue=crudo, solido=mediana)')
    ax_rgb.legend(loc='upper left', fontsize=8, ncol=3)
    ax_rgb.grid(True, alpha=0.25)

    ax_hsv.set_ylim(0, 255)
    ax_hsv.set_xlabel('cuadro')
    ax_hsv.set_ylabel('H:0-179 / S,V:0-255, promedio del ROI')
    ax_hsv.set_title('HSV - ROI sorter (tenue=crudo, solido=mediana)')
    ax_hsv.legend(loc='upper left', fontsize=8, ncol=3)
    ax_hsv.grid(True, alpha=0.25)

    blank = np.zeros((10, 10, 3), dtype=np.uint8)
    cam_image = ax_cam.imshow(blank)
    ax_cam.set_title('camara - sorter (ROI + HSV vs RGB)', fontsize=10)
    ax_cam.set_xticks([])
    ax_cam.set_yticks([])

    fig.suptitle('ROI del sorter - cuadro a cuadro (HSV real vs RGB nuevo, en paralelo)')
    fig.tight_layout()

    REASON_LABEL = {
        "no ball": "sin pelota",
        "green -> MATURE": "VERDE -> MADURA",
        "warm -> MATURE": "CALIENTE -> MADURA",
        "cool -> IMMATURE": "FRIO -> INMADURA",
        "ambiguous (warm+cool)": "AMBIGUO (caliente+frio)",
        "unknown -> MATURE (fallback)": "desconocido -> MADURA",
        "unknown -> NEUTRAL (background?)": "desconocido -> NEUTRAL",
    }

    def update(_frame):
        ok, frame = cap.read()
        if not ok:
            return []

        roi = frame[ry:ry + rh, rx:rx + rw]
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mean_bgr = roi.reshape(-1, 3).mean(axis=0)
        mean_hsv = roi_hsv.reshape(-1, 3).mean(axis=0)

        t.append(frame_idx[0])
        frame_idx[0] += 1
        buf['R'].append(float(mean_bgr[2])); buf['G'].append(float(mean_bgr[1])); buf['B'].append(float(mean_bgr[0]))
        buf['H'].append(float(mean_hsv[0])); buf['S'].append(float(mean_hsv[1])); buf['V'].append(float(mean_hsv[2]))

        n = args.median_n
        med = {ch: median_last_n(buf[ch], n) for ch in buf}
        med_bgr = np.array([med['B'], med['G'], med['R']])
        med_hsv = np.array([med['H'], med['S'], med['V']])

        # classify() de HSV trabaja sobre una IMAGEN (hace inRange por
        # pixel), no sobre un solo valor -- se le pasa un ROI sintetico
        # uniforme con el color ya filtrado por mediana, asi se reusa
        # classify()/masks_for() sin tocarlos.
        synthetic_hsv = np.full_like(roi_hsv, med_hsv.astype(np.uint8))
        pct = {name: sx._pct(m, roi_pixels) for name, m in sx.masks_for(synthetic_hsv, cfg).items()}
        _, _, _, reason = sx.classify(pct, cfg)
        label_hsv = REASON_LABEL.get(reason, reason)

        rgb_color, rgb_bin, _ = sx.classify_rgb_mean(med_bgr)
        label_rgb = f"{rgb_color.upper()} -> {rgb_bin}"

        xs = list(t)
        for ch in ('R', 'G', 'B'):
            lines_rgb[ch].set_data(xs, list(buf[ch]))
            lines_rgb_med[ch].set_data(xs, _median_series(buf[ch], n))
        for ch in ('H', 'S', 'V'):
            lines_hsv[ch].set_data(xs, list(buf[ch]))
            lines_hsv_med[ch].set_data(xs, _median_series(buf[ch], n))
        if xs:
            xlim = (max(0, xs[0]), max(xs[-1], xs[0] + 1))
            ax_rgb.set_xlim(*xlim)
            ax_hsv.set_xlim(*xlim)

        annotated = frame.copy()
        cv2.rectangle(annotated, (rx, ry), (rx + rw, ry + rh), (255, 180, 0), 2)
        cv2.putText(annotated, f'HSV: {label_hsv}', (rx, max(24, ry - 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 255, 120), 2, cv2.LINE_AA)
        cv2.putText(annotated, f'RGB: {label_rgb}', (rx, max(24, ry - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 200, 255), 2, cv2.LINE_AA)
        cam_image.set_data(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
        cam_image.set_extent((0, annotated.shape[1], annotated.shape[0], 0))
        ax_cam.set_xlim(0, annotated.shape[1])
        ax_cam.set_ylim(annotated.shape[0], 0)

        return (list(lines_rgb.values()) + list(lines_rgb_med.values())
                + list(lines_hsv.values()) + list(lines_hsv_med.values()) + [cam_image])

    ani = matplotlib.animation.FuncAnimation(fig, update, interval=args.interval_ms, blit=False)

    plt.show()
    cap.release()


if __name__ == '__main__':
    main()
