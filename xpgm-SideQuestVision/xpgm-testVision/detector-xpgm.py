#!/usr/bin/env python3
"""Prototipo offline en xpgm-testVision: corrige el supuesto de geometria de
vision_side_quest_xpgm/detector_pelotas/detector.py para montaje VERTICAL de
la ZED, donde cada ojo del par estereo ve una zona COMPLETA y distinta (no
ambas zonas apiladas dentro de la misma vista).

Diferencia con el detector real: alli, --sbs-both corre detect() dos veces
(una por mitad) con las MISMAS dos sub-regiones (arriba/abajo) dentro de cada
mitad, y junta resultados por nombre de zona -- eso asume que cada ojo ve las
dos filas apiladas. Aqui, cada mitad entera ES una sola zona (sin sub-dividir)
segun cfg['eye_zones']; confirmado con el usuario: ojo izquierdo = abajo, ojo
derecho = arriba, por como quedo el montaje vertical.

NO modifica ni importa nada de vision_side_quest_xpgm: _detect_by_hue,
_detect_by_edges y Confirmation estan copiados sin cambios de detector.py
(misma logica de deteccion de color/bordes), solo cambia como se arman las
zonas a partir de las dos mitades.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DEFAULT = {
    "width": 640,
    "eye_zones": {"left": "abajo", "right": "arriba"},
    # El sensor de cada ojo sigue entregando un cuadro horizontal (ancho >
    # alto) aunque la barra ZED este girada para quedar vertical: el contenido
    # real queda acostado 90 grados dentro de ese cuadro. rotate_deg gira el
    # recorte de CADA ojo antes de procesarlo/mostrarlo (no cambia cual ojo es
    # arriba/abajo, eso ya lo resuelve eye_zones). Confirmado -90 (sentido
    # antihorario) contra el robot real; 90 (horario) quedaba al reves.
    "rotate_deg": -90,
    # max_radius subido de 70 a 160: de cerca (la distancia real al arbol)
    # la pelota llena mucho mas del cuadro de lo que este calibrado original
    # permitia, y detect_single_zone() la descartaba por completo.
    "min_radius": 40, "max_radius": 160,
    # "azul" agregado: sin el no habia ningun rango que lo cubriera, asi que
    # _detect_by_hue (el metodo barato, por mascara de color) nunca lo
    # buscaba -- solo lo encontraba _detect_by_edges (circulos), mas
    # sensible a sombra/bajo contraste. Con esto hay una segunda via de
    # deteccion independiente.
    # "verde" ensanchado de [40,100] a [35,105]: con min_ball_saturation mas
    # bajo (45) entran lecturas de menor saturacion, y a baja saturacion el
    # tono (H) es mas ruidoso -- una pelota verde real podia leer H=36 o
    # H=103 y caer fuera del rango original, clasificando como "otro" (osea
    # SI recogible). "no verdes" es regla dura, mejor un rango mas tolerante
    # ahi que arriesgarse a recoger una verde. "azul" se recorta a partir de
    # 106 para no traslaparse.
    "ball_colors": {"rojo": [[0, 10], [170, 179]],
                     "verde": [[35, 105]],
                     "azul": [[106, 140]]},
    # min_ball_saturation bajado de 90 a 45: la sombra del propio robot cae
    # sobre la pelota (sobre todo la azul) y le baja la saturacion -- con 90
    # _detect_by_edges la descartaba por completo (no solo el color, el
    # candidato entero), dando "sin_pelota" aunque la pelota si estuviera ahi.
    "min_ball_saturation": 45, "min_ball_value": 15,
    "min_fill": 0.5,
    "edge_threshold": 100, "hough_votes": 24,
    "min_arc_support": 0.60, "min_sectors": 6,
    "confirm_frames": 3, "max_gap_ms": 250
}


def _detect_by_hue(img, hsv, cfg):
    """Copiado sin cambios de detector_pelotas/detector.py."""
    h, w = img.shape[:2]
    kernel = np.ones((3, 3), np.uint8)
    out = []
    for color_name, hue_ranges in cfg['ball_colors'].items():
        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in hue_ranges:
            lower = (lo, cfg['min_ball_saturation'], cfg['min_ball_value'])
            upper = (hi, 255, 255)
            mask |= cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for ct in contours:
            area = cv2.contourArea(ct)
            (x, y), radius = cv2.minEnclosingCircle(ct)
            if not (cfg['min_radius'] <= radius <= cfg['max_radius']):
                continue
            bx, by, bw, bh = cv2.boundingRect(ct)
            if bx <= 0 or by <= 0 or bx + bw >= w or by + bh >= h:
                continue
            circ_area = np.pi * radius * radius
            fill = area / circ_area if circ_area > 0 else 0.0
            if fill < cfg['min_fill']:
                continue
            out.append({'x': float(x), 'y': float(y), 'r': float(radius),
                        'support': float(fill), 'color': color_name})
    return out


def _detect_by_edges(img, hsv, cfg):
    """Copiado sin cambios de detector_pelotas/detector.py."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2, tileGridSize=(8, 8)).apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    threshold = cfg['edge_threshold']
    edges = cv2.Canny(gray, threshold / 2, threshold)
    distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2,
        minDist=cfg['min_radius'] * 1.8,
        param1=threshold, param2=cfg['hough_votes'],
        minRadius=cfg['min_radius'], maxRadius=cfg['max_radius'])
    h, w = gray.shape
    angles = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    out = []
    if circles is None:
        return out
    for x, y, radius in circles[0]:
        if x - radius < 0 or y - radius < 0 or x + radius >= w or y + radius >= h:
            continue
        xs = np.rint(x + radius * np.cos(angles)).astype(int)
        ys = np.rint(y + radius * np.sin(angles)).astype(int)
        # XPGM FIX: np.rint can round a sample right onto the frame's edge
        # (e.g. 357.6 passes the x+radius>=w check above, then rounds to
        # 358 on a width-358 array -> IndexError). Only matters at larger
        # max_radius; clipping doesn't change which circles get accepted,
        # just stops an edge sample from reading one pixel past the array.
        xs = np.clip(xs, 0, w - 1)
        ys = np.clip(ys, 0, h - 1)
        hits = distance[ys, xs] <= max(2.0, 0.06 * radius)
        support = float(hits.mean())
        sectors = int((hits.reshape(12, 10).mean(axis=1) >= 0.5).sum())
        if support < cfg['min_arc_support'] or sectors < cfg['min_sectors']:
            continue
        inner = max(2, round(radius * 0.5))
        y0s, y1s = max(0, round(y - inner)), min(h, round(y + inner))
        x0s, x1s = max(0, round(x - inner)), min(w, round(x + inner))
        patch = hsv[y0s:y1s, x0s:x1s].reshape(-1, 3)
        if patch.size == 0:
            continue
        med_h = float(np.median(patch[:, 0]))
        med_s = float(np.median(patch[:, 1]))
        if med_s < cfg['min_ball_saturation']:
            continue
        color = 'otro'
        for color_name, hue_ranges in cfg['ball_colors'].items():
            if any(lo <= med_h <= hi for lo, hi in hue_ranges):
                color = color_name
                break
        out.append({'x': float(x), 'y': float(y), 'r': float(radius),
                    'support': support, 'color': color})
    return out


_ROTATE_FLAG = {90: cv2.ROTATE_90_CLOCKWISE, -90: cv2.ROTATE_90_COUNTERCLOCKWISE,
                 270: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180}


def detect_single_zone(frame, cfg):
    """Una mitad entera = una zona completa (sin sub-dividir arriba/abajo
    dentro de la vista): arcos primero, color solo si arcos no encontro NADA
    en todo el cuadro (misma prioridad que el detector real, pero sin el
    filtrado por sub-region porque aqui ya no hay sub-region)."""
    # Reescalar ANTES de rotar: el ancho original (recorte horizontal del
    # sensor) es a lo que esta calibrado min_radius/max_radius. Si se rota
    # primero, el "ancho" pasa a ser la altura original (~376px en vez de
    # ~672px), el factor de escala hacia cfg['width'] se dispara (~1.7x en
    # vez de ~0.95x), todo se ve mas grande de lo calibrado, y la pelota real
    # se sale de max_radius mientras un blob de sombra entra como falso
    # positivo. Rotar despues de reescalar mantiene la escala pelota-a-pixel
    # calibrada en banco, solo gira el resultado ya escalado.
    height, width = frame.shape[:2]
    scale = cfg['width'] / width
    img = cv2.resize(frame, (cfg['width'], round(height * scale)))
    rotate_deg = cfg.get('rotate_deg', 0)
    if rotate_deg in _ROTATE_FLAG:
        img = cv2.rotate(img, _ROTATE_FLAG[rotate_deg])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    found = _detect_by_edges(img, hsv, cfg)
    if not found:
        found = _detect_by_hue(img, hsv, cfg)
    found.sort(key=lambda c: c['support'], reverse=True)
    return img, found


def annotate(image, found, zone_name, state_value):
    h, w = image.shape[:2]
    cv2.rectangle(image, (2, 2), (w - 2, h - 2), (255, 180, 0), 1)
    cv2.putText(image, f'{zone_name}: {state_value}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    for c in found:
        color = (0, 255, 0) if c.get('color') == 'verde' else (0, 255, 255)
        cv2.circle(image, (round(c['x']), round(c['y'])), round(c['r']), color, 2)
    return image


class Confirmation:
    """Copiado sin cambios de detector_pelotas/detector.py (misma logica de
    continuidad entre cuadros), solo con las claves que definan eye_zones."""
    def __init__(self, zone_names, cfg):
        self.cfg = cfg
        self.state = {name: {'last': None, 'count': 0, 'time': None} for name in zone_names}

    def update(self, detections, now):
        output = {}
        for name, state in self.state.items():
            candidates = detections.get(name, [])
            last = state['last']
            fresh = state['time'] is not None and (now - state['time']) * 1000 <= self.cfg['max_gap_ms']
            matches = [c for c in candidates if last is not None and
                       np.hypot(c['x'] - last['x'], c['y'] - last['y']) <= 0.6 * last['r'] and
                       abs(c['r'] - last['r']) <= 0.3 * last['r']]
            chosen = matches[0] if fresh and matches else (candidates[0] if candidates else None)
            if chosen is None:
                state.update(last=None, count=0, time=now)
            else:
                continuing = fresh and bool(matches)
                state.update(last=chosen, count=state['count'] + 1 if continuing else 1, time=now)
            output[name] = int(state['count'] >= self.cfg['confirm_frames'])
        return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', default='0', help='Indice UVC o ruta de video')
    p.add_argument('--image', help='Analizar una imagen; no aplica confirmacion temporal')
    p.add_argument('--config', help='JSON de configuracion (ver DEFAULT)')
    p.add_argument('--write-config', help='Guardar configuracion inicial y salir')
    p.add_argument('--preview', action='store_true', help='Requiere OpenCV con GUI y escritorio')
    p.add_argument('--preview-scale', type=float, default=0.6,
                    help='Factor de escala SOLO de la ventana en pantalla (no toca deteccion ni --save)')
    p.add_argument('--save', help='Guardar imagen anotada en modo --image')
    args = p.parse_args()

    cfg = json.loads(Path(args.config).read_text()) if args.config else DEFAULT
    if args.write_config:
        Path(args.write_config).write_text(json.dumps(cfg, indent=2) + '\n')
        return
    if not 0 < cfg['min_radius'] < cfg['max_radius']:
        p.error('Radios invalidos')

    zone_names = list(cfg['eye_zones'].values())
    if len(set(zone_names)) != 2:
        p.error("eye_zones debe asignar 'left' y 'right' a dos zonas DISTINTAS")

    cap = None
    tracker = Confirmation(zone_names, cfg)
    sequence = 0
    try:
        if not args.image:
            source = int(args.source) if args.source.isdecimal() else args.source
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                raise RuntimeError('No se pudo abrir la camara o video')
        while True:
            if args.image:
                frame = cv2.imread(args.image)
                ok = frame is not None
            else:
                ok, frame = cap.read()
            if not ok:
                print(json.dumps({'valid': False, 'reason': 'sin_imagen', 'seq': sequence}), flush=True)
                break

            w = frame.shape[1]
            image_left, found_left = detect_single_zone(frame[:, :w // 2], cfg)
            image_right, found_right = detect_single_zone(frame[:, w // 2:], cfg)

            zone_left = cfg['eye_zones']['left']
            zone_right = cfg['eye_zones']['right']
            found = {zone_left: found_left, zone_right: found_right}

            now = time.monotonic()
            pickable = {name: [c for c in found[name] if c['color'] != 'verde'] for name in found}
            state = tracker.update(pickable, now)

            packet = {'valid': True, 'seq': sequence, 't_monotonic_ms': round(now * 1000),
                      'arriba': state.get('arriba', 0), 'abajo': state.get('abajo', 0),
                      'candidates': found}
            if args.image:
                packet['mode'] = 'imagen_sin_confirmacion_temporal'
                packet['arriba'] = packet['abajo'] = None
            print(json.dumps(packet), flush=True)

            if args.preview or args.save:
                image_left = annotate(image_left, found_left, zone_left, state.get(zone_left))
                image_right = annotate(image_right, found_right, zone_right, state.get(zone_right))
                by_zone = {zone_left: image_left, zone_right: image_right}
                top, bottom = by_zone['arriba'], by_zone['abajo']
                if top.shape[1] != bottom.shape[1] or top.shape[0] != bottom.shape[0]:
                    bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
                image = np.vstack([top, bottom])
                if args.save and args.image:
                    if not cv2.imwrite(args.save, image):
                        raise RuntimeError('No se pudo guardar la imagen')
                if args.preview:
                    shown = image
                    if args.preview_scale != 1.0:
                        shown = cv2.resize(
                            image,
                            (round(image.shape[1] * args.preview_scale), round(image.shape[0] * args.preview_scale)))
                    cv2.imshow('Geometria (xpgm, ojo=zona completa)', shown)
                    if cv2.waitKey(1) & 255 == ord('q'):
                        break
            if args.image:
                break
            sequence += 1
    finally:
        if cap is not None:
            cap.release()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
