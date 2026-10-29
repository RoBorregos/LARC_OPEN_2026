#!/usr/bin/env python3
"""Dashboard en vivo tipo Serial Plotter (Arduino) para xpgm-testVision: UNA
sola ventana de matplotlib, grid de 2 filas x 3 columnas -- arriba arriba,
abajo abajo, y de izquierda a derecha RGB | camara | HSV. La columna de
camara es la misma vista anotada (zona + circulos) que detector-xpgm.py
--preview, embebida aqui como imagen en vez de una ventana cv2 aparte.

Reusa detect_single_zone()/annotate()/DEFAULT de detector-xpgm.py tal cual
(mismo import que sample-colors-xpgm.py) -- no duplica logica de deteccion,
no toca nada de vision_side_quest_xpgm.

Requiere escritorio (igual que --preview en detector-xpgm.py): correr desde
el escritorio remoto, no por SSH puro.
"""
import argparse
import importlib.util
import statistics
from collections import deque
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np


def load_detector_module():
    here = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location("detector_xpgm", here / "detector-xpgm.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def best_candidate(found):
    return found[0] if found else None


def _valid(vals):
    return [v for v in vals if not (isinstance(v, float) and np.isnan(v))]


def median_last_n(dq, n):
    """x(t) = median(x(t-1), ..., x(t-n)) -- sugerencia del Dr. Bortoni para
    no decidir el color con un solo cuadro ruidoso. Ignora los huecos (NaN,
    cuadros sin candidato) dentro de la ventana en vez de contarlos."""
    window = _valid(list(dq)[-n:])
    return statistics.median(window) if window else float('nan')


def _median_series(dq, n):
    """Mediana movil sobre todo el buffer, para la linea de mediana en el
    plot. Requiere al menos 1 muestra valida en la ventana (los huecos son
    comunes aqui, a diferencia del sorter donde siempre hay ROI)."""
    vals = list(dq)
    out = []
    for i in range(len(vals)):
        window = _valid(vals[max(0, i - n + 1):i + 1])
        out.append(statistics.median(window) if window else float('nan'))
    return out


def reclassify_hue(h, s, v, cfg):
    """Mismo filtro de saturacion/valor + rangos de ball_colors que ya usa
    _detect_by_edges/_detect_by_hue en detector.py, pero aplicado al H/S/V
    YA filtrado por mediana en vez del cuadro crudo."""
    if s < cfg['min_ball_saturation'] or v < cfg['min_ball_value']:
        return 'otro'
    for color_name, hue_ranges in cfg['ball_colors'].items():
        if any(lo <= h <= hi for lo, hi in hue_ranges):
            return color_name
    return 'otro'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', default='/dev/video_zed', help='Indice UVC o ruta de video')
    p.add_argument('--window', type=int, default=150, help='Cuadros visibles en la ventana deslizante')
    p.add_argument('--interval-ms', type=int, default=150, help='Intervalo de refresco del plot (ms)')
    p.add_argument('--median-n', type=int, default=5,
                    help='Ventana del filtro de mediana que decide el color (sugerencia Dr. Bortoni)')
    args = p.parse_args()

    det = load_detector_module()
    cfg = det.DEFAULT
    zone_left = cfg['eye_zones']['left']
    zone_right = cfg['eye_zones']['right']
    eye_of_zone = {zone_left: 'left', zone_right: 'right'}
    # Orden fijo arriba-primero para que coincida con el montaje vertical.
    zone_order = [z for z in ('arriba', 'abajo') if z in (zone_left, zone_right)]

    source = int(args.source) if args.source.isdecimal() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError('No se pudo abrir la camara o video: ' + str(args.source))

    N = args.window
    t = deque(maxlen=N)
    buf = {zone: {ch: deque(maxlen=N) for ch in ('R', 'G', 'B', 'H', 'S', 'V')} for zone in zone_order}
    frame_idx = [0]

    chan_color = {'R': '#d03b3b', 'G': '#0ca30c', 'B': '#2a78d6',
                  'H': '#9b59b6', 'S': '#e67e22', 'V': '#7f8c8d'}

    # Zona de "llegada" (la puerta que dibuja la linea horizontal acortada):
    # en fraccion de cuadro (0-1), no en pixeles, para no depender del tamano
    # real. GATE_X debe coincidir con el segmento dibujado ([0.35, 0.65]).
    # GATE_Y_TOL es un primer numero razonable, no calibrado con el montaje
    # final -- ajustar viendo si BAJAR dispara ni muy pronto ni muy tarde.
    GATE_X = (0.35, 0.65)
    GATE_Y_TOL = 0.08

    fig = plt.figure(figsize=(13, 7.5))
    fig.canvas.manager.set_window_title('Dashboard en vivo (xpgm) - RGB | camara | HSV')
    gs = gridspec.GridSpec(2, 3, figure=fig, width_ratios=[1.1, 0.9, 1.1], hspace=0.3, wspace=0.28)

    ax_rgb, ax_hsv, ax_cam = {}, {}, {}
    for row, zone in enumerate(zone_order):
        ax_rgb[zone] = fig.add_subplot(gs[row, 0])
        ax_cam[zone] = fig.add_subplot(gs[row, 1])
        ax_hsv[zone] = fig.add_subplot(gs[row, 2])

    # Cruda = delgada/tenue, mediana = gruesa/solida (lo que decide el color
    # de verdad) -- mismo lenguaje visual que dashboardSorter-xpgm.py.
    lines_rgb, lines_rgb_med, lines_hsv, lines_hsv_med, cam_images = {}, {}, {}, {}, {}
    for zone in zone_order:
        for ch in ('R', 'G', 'B'):
            (line,) = ax_rgb[zone].plot([], [], '-', color=chan_color[ch], alpha=0.35, linewidth=1.1)
            lines_rgb[(zone, ch)] = line
            (line_med,) = ax_rgb[zone].plot([], [], '-', color=chan_color[ch], label=ch, linewidth=1.8)
            lines_rgb_med[(zone, ch)] = line_med
        ax_rgb[zone].set_ylim(0, 255)
        ax_rgb[zone].set_ylabel('canal (0-255)')
        ax_rgb[zone].set_title(f'RGB - {zone} (tenue=crudo, solido=mediana)', loc='left', fontsize=9)
        ax_rgb[zone].legend(loc='upper left', fontsize=8, ncol=3)
        ax_rgb[zone].grid(True, alpha=0.25)

        for ch in ('H', 'S', 'V'):
            (line,) = ax_hsv[zone].plot([], [], '-', color=chan_color[ch], alpha=0.35, linewidth=1.1)
            lines_hsv[(zone, ch)] = line
            (line_med,) = ax_hsv[zone].plot([], [], '-', color=chan_color[ch], label=ch, linewidth=1.8)
            lines_hsv_med[(zone, ch)] = line_med
        ax_hsv[zone].set_ylim(0, 255)
        ax_hsv[zone].set_ylabel('H:0-179 / S,V:0-255')
        ax_hsv[zone].set_title(f'HSV - {zone} (tenue=crudo, solido=mediana)', loc='left', fontsize=9)
        ax_hsv[zone].legend(loc='upper left', fontsize=8, ncol=3)
        ax_hsv[zone].grid(True, alpha=0.25)

        blank = np.zeros((10, 10, 3), dtype=np.uint8)
        cam_images[zone] = ax_cam[zone].imshow(blank)
        ax_cam[zone].set_title(f'camara - {zone}', fontsize=10)
        ax_cam[zone].set_xticks([])
        ax_cam[zone].set_yticks([])

        # Reticula fija en el centro: punto de referencia al que debe llegar
        # la pelota antes de bajar el servo de esa zona. En coordenadas de
        # ejes (0-1), asi que queda centrada sin importar el tamano real del
        # cuadro, y no hay que redibujarla cuadro a cuadro.
        # axhline/axvline no aceptan transform en esta version de matplotlib
        # (3.5.1) -- plot() si, y da el mismo resultado en coordenadas de ejes.
        # Horizontal acortada (solo 30% central): es la "puerta" de llegada,
        # no una linea de borde a borde -- marca donde debe estar la pelota,
        # no cualquier altura. Vertical se deja completa como referencia de
        # centrado lateral. Puramente visual por ahora: NO filtra todavia la
        # decision BAJAR/no bajar por posicion, solo por color (ver mas abajo).
        ax_cam[zone].plot([0.35, 0.65], [0.5, 0.5], transform=ax_cam[zone].transAxes,
                           color='yellow', linestyle='--', linewidth=1.6, alpha=0.9, zorder=5)
        ax_cam[zone].plot([0.5, 0.5], [0, 1], transform=ax_cam[zone].transAxes,
                           color='yellow', linestyle='--', linewidth=1.3, alpha=0.85, zorder=5)

    ax_rgb[zone_order[-1]].set_xlabel('cuadro')
    ax_hsv[zone_order[-1]].set_xlabel('cuadro')
    fig.suptitle('Mejor candidato por zona - cuadro a cuadro')

    def update(_frame):
        ok, frame = cap.read()
        if not ok:
            return []
        w = frame.shape[1]
        halves = {'left': frame[:, :w // 2], 'right': frame[:, w // 2:]}

        t.append(frame_idx[0])
        frame_idx[0] += 1

        n = args.median_n
        for zone in zone_order:
            crop = halves[eye_of_zone[zone]]
            img, found = det.detect_single_zone(crop, cfg)
            c = best_candidate(found)

            if c is None:
                for ch in buf[zone]:
                    buf[zone][ch].append(np.nan)
                decision = 'sin pelota'
            else:
                x, y = round(c['x']), round(c['y'])
                x = min(max(x, 0), img.shape[1] - 1)
                y = min(max(y, 0), img.shape[0] - 1)
                bgr = img[y, x]
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[y, x]
                buf[zone]['R'].append(int(bgr[2]))
                buf[zone]['G'].append(int(bgr[1]))
                buf[zone]['B'].append(int(bgr[0]))
                buf[zone]['H'].append(int(hsv[0]))
                buf[zone]['S'].append(int(hsv[1]))
                buf[zone]['V'].append(int(hsv[2]))

                # Decision de servo de ESTE cuadro, con el COLOR pasado por
                # mediana de n=5 (sugerencia Dr. Bortoni) en vez del color
                # crudo de _detect_by_edges/_detect_by_hue -- un cuadro
                # aislado raro ya no cambia el veredicto. La posicion (fx,fy)
                # para la puerta NO se filtra: es "donde esta ahora", filtrar
                # posicion solo meteria retraso donde no hace falta. BAJAR
                # sigue exigiendo las DOS condiciones: color no-verde Y
                # centro dentro de la puerta. Esto sigue sin el debounce de
                # confirm_frames del pipeline real -- la mediana baja el
                # ruido de UN cuadro, no reemplaza esa confirmacion temporal.
                med_h = median_last_n(buf[zone]['H'], n)
                med_s = median_last_n(buf[zone]['S'], n)
                med_v = median_last_n(buf[zone]['V'], n)
                color = reclassify_hue(med_h, med_s, med_v, cfg)

                if color == 'verde':
                    decision = 'VERDE -> no bajar'
                else:
                    fx = c['x'] / img.shape[1]
                    fy = c['y'] / img.shape[0]
                    in_gate = GATE_X[0] <= fx <= GATE_X[1] and abs(fy - 0.5) <= GATE_Y_TOL
                    if in_gate:
                        decision = f"{color.upper()} -> BAJAR"
                    else:
                        # Texto corto a proposito: a font scale 0.6 un
                        # mensaje largo se sale del ancho angosto del
                        # panel (~358px).
                        decision = f"{color.upper()} (esperando)"

            annotated = det.annotate(img.copy(), found, zone, decision)
            cam_images[zone].set_data(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
            cam_images[zone].set_extent((0, annotated.shape[1], annotated.shape[0], 0))
            ax_cam[zone].set_xlim(0, annotated.shape[1])
            ax_cam[zone].set_ylim(annotated.shape[0], 0)

        xs = list(t)
        for zone in zone_order:
            for ch in ('R', 'G', 'B'):
                lines_rgb[(zone, ch)].set_data(xs, list(buf[zone][ch]))
                lines_rgb_med[(zone, ch)].set_data(xs, _median_series(buf[zone][ch], n))
            for ch in ('H', 'S', 'V'):
                lines_hsv[(zone, ch)].set_data(xs, list(buf[zone][ch]))
                lines_hsv_med[(zone, ch)].set_data(xs, _median_series(buf[zone][ch], n))

        if xs:
            xlim = (max(0, xs[0]), max(xs[-1], xs[0] + 1))
            for zone in zone_order:
                ax_rgb[zone].set_xlim(*xlim)
                ax_hsv[zone].set_xlim(*xlim)

        artists = (list(lines_rgb.values()) + list(lines_rgb_med.values())
                   + list(lines_hsv.values()) + list(lines_hsv_med.values())
                   + list(cam_images.values()))
        return artists

    ani = matplotlib.animation.FuncAnimation(fig, update, interval=args.interval_ms, blit=False)

    plt.show()
    cap.release()


if __name__ == '__main__':
    main()
