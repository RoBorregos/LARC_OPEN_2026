#!/usr/bin/env python3
"""Prototipo geométrico offline. Ver README.md antes de conectar al robot."""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DEFAULT = {
    "width": 640,
    "regions": {"arriba": [0.05, 0.08, 0.95, 0.49],
                "abajo": [0.05, 0.51, 0.95, 0.95]},
    # Pelotas reales medidas en banco: 56-65px a esta distancia (roja y verde,
    # por ambos métodos). Un cable de la cámara midió 21px y pasaba como falso
    # positivo con min_radius=12 — 40 lo excluye con margen a ambos lados. Es
    # específico de ESTA distancia de banco: recalibrar con el montaje final,
    # el tamaño aparente de la pelota cambia con la distancia a la cámara.
    "min_radius": 40, "max_radius": 70,
    # Matiz en escala OpenCV (H: 0-179). "rojo" usa dos rangos porque el rojo
    # cruza el cero del círculo de matiz (179 y 0 son el mismo color). Se usa
    # para segmentar por color Y para clasificar candidatos encontrados por borde.
    "ball_colors": {"rojo": [[0, 10], [170, 179]],
                     "verde": [[40, 100]]},
    "min_ball_saturation": 90, "min_ball_value": 15,
    "min_fill": 0.5,
    "edge_threshold": 100, "hough_votes": 24,
    "min_arc_support": 0.60, "min_sectors": 6,
    "confirm_frames": 3, "max_gap_ms": 250
}


def _detect_by_hue(img, hsv, cfg):
    """Pelotas separables del fondo por matiz, incluso con poco contraste de
    brillo (p. ej. roja en sombra: soporte de borde 0.16-0.54, nunca cruza el
    umbral real, pero el matiz se mantiene estable). FALLA si el color de la
    pelota coincide con el del fondo: confirmado en banco, una pelota verde
    sobre campo verde se funde en un solo blob con el fondo (bbox de ~590x350
    sobre un cuadro de 640x358, circularidad 0.34, no pasa min_fill) — no hay
    forma de separar por matiz lo que ya comparte matiz con el fondo. Por eso
    nunca reemplaza a _detect_by_edges, corre en paralelo con ella.
    """
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
            # Se usa el cuadro del CONTORNO real, no el círculo ajustado: minEnclosingCircle
            # puede sobrepasar el borde por imprecisión del ajuste aunque el blob de color
            # esté completo y visible — comprobado en banco con la pelota roja real.
            bx, by, bw, bh = cv2.boundingRect(ct)
            if bx <= 0 or by <= 0 or bx+bw >= w or by+bh >= h:
                continue  # blob de color cortado por el borde: no confirmar geometría inventada
            circ_area = np.pi * radius * radius
            fill = area / circ_area if circ_area > 0 else 0.0
            if fill < cfg['min_fill']:
                continue  # forma alargada (p. ej. un cable) o fundida con el fondo
            out.append({'x': float(x), 'y': float(y), 'r': float(radius),
                        'support': float(fill), 'color': color_name})
    return out


def _detect_by_edges(img, hsv, cfg):
    """Pelotas separables del fondo por brillo/borde (Hough), clasificadas por
    color con una muestra de matiz/saturación en su interior. Resuelve el caso
    que _detect_by_hue no puede (color igual al del fondo, p. ej. la pelota
    verde: soporte 0.62-0.73 medido en banco). FALLA cuando el contraste de
    brillo es pobre (sombra fuerte) — ese es el caso que sí resuelve
    _detect_by_hue. Nunca reemplaza a esa función, corre en paralelo con ella.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2, tileGridSize=(8, 8)).apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    threshold = cfg['edge_threshold']
    edges = cv2.Canny(gray, threshold / 2, threshold)
    # Distancia al borde más cercano: tolera pequeñas imprecisiones del radio.
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
        if x-radius < 0 or y-radius < 0 or x+radius >= w or y+radius >= h:
            continue
        xs = np.rint(x + radius * np.cos(angles)).astype(int)
        ys = np.rint(y + radius * np.sin(angles)).astype(int)
        hits = distance[ys, xs] <= max(2.0, 0.06 * radius)
        support = float(hits.mean())
        sectors = int((hits.reshape(12, 10).mean(axis=1) >= 0.5).sum())
        if support < cfg['min_arc_support'] or sectors < cfg['min_sectors']:
            continue
        # Color: muestra el interior del candidato para clasificarlo. Si no
        # alcanza saturación de pelota, se descarta aunque haya pasado soporte
        # de borde (así se eliminaron el soporte blanco y los dientes del
        # separador sin arriesgar el umbral de soporte).
        inner = max(2, round(radius * 0.5))
        y0s, y1s = max(0, round(y-inner)), min(h, round(y+inner))
        x0s, x1s = max(0, round(x-inner)), min(w, round(x+inner))
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


def detect(frame, cfg):
    """Arcos (_detect_by_edges, Hough) es el método PRINCIPAL, como se pidió desde
    el inicio: centro, radio y apoyo de bordes alrededor de la circunferencia.
    _detect_by_hue (color) no es un método paralelo de igual peso: solo aporta
    candidatos en una zona cuando los arcos no encontraron NADA ahí en ese
    cuadro (p. ej. pelota en sombra fuerte, sin borde suficiente en gris). Si
    los arcos ya ven algo en la zona, el color no agrega ni reemplaza nada ahí
    — solo clasifica (verde o no) lo que los arcos ya encontraron."""
    height, width = frame.shape[:2]
    scale = cfg['width'] / width
    img = cv2.resize(frame, (cfg['width'], round(height * scale)))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, w = img.shape[:2]
    result = {name: [] for name in cfg['regions']}
    for c in _detect_by_edges(img, hsv, cfg):
        for name, (x0, y0, x1, y1) in cfg['regions'].items():
            if x0*w <= c['x'] < x1*w and y0*h <= c['y'] < y1*h:
                result[name].append(c)
    for c in _detect_by_hue(img, hsv, cfg):
        for name, (x0, y0, x1, y1) in cfg['regions'].items():
            if x0*w <= c['x'] < x1*w and y0*h <= c['y'] < y1*h and not result[name]:
                result[name].append(c)
    for name in result:
        result[name].sort(key=lambda c: c['support'], reverse=True)
    return img, result


def annotate(image, found, state, cfg):
    """Dibuja zonas, estado y círculos candidatos sobre una vista (una sola mitad o imagen completa)."""
    h, w = image.shape[:2]
    for name, region in cfg['regions'].items():
        x0, y0, x1, y1 = region
        cv2.rectangle(image, (round(x0*w), round(y0*h)), (round(x1*w), round(y1*h)), (255, 180, 0), 1)
        cv2.putText(image, f'{name}: {state[name]}', (round(x0*w), round(y0*h)+18), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        for c in found[name]:
            color = (0, 255, 0) if c.get('color') == 'verde' else (0, 255, 255)
            cv2.circle(image, (round(c['x']), round(c['y'])), round(c['r']), color, 2)
    return image


class Confirmation:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = {name: {'last': None, 'count': 0, 'time': None}
                      for name in cfg['regions']}

    def update(self, detections, now):
        output = {}
        for name, state in self.state.items():
            candidates = detections[name]
            last = state['last']
            fresh = state['time'] is not None and (now-state['time'])*1000 <= self.cfg['max_gap_ms']
            matches = [c for c in candidates if last is not None and
                       np.hypot(c['x']-last['x'], c['y']-last['y']) <= 0.6*last['r'] and
                       abs(c['r']-last['r']) <= 0.3*last['r']]
            chosen = matches[0] if fresh and matches else (candidates[0] if candidates else None)
            if chosen is None:
                state.update(last=None, count=0, time=now)
            else:
                continuing = fresh and bool(matches)
                state.update(last=chosen, count=state['count']+1 if continuing else 1, time=now)
            output[name] = int(state['count'] >= self.cfg['confirm_frames'])
        return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', default='0', help='Índice UVC o ruta de video')
    p.add_argument('--sbs-left', action='store_true', help='Recortar mitad izquierda SOLO si UVC entrega estéreo lado a lado')
    p.add_argument('--sbs-both', action='store_true', help='Procesar ambas mitades (izquierda y derecha) y fusionar candidatos por zona; usar cuando a la distancia de montaje una pelota puede caer fuera del campo de un solo ojo')
    p.add_argument('--image', help='Analizar una imagen; no aplica confirmación temporal')
    p.add_argument('--config', help='JSON guardado con --write-config')
    p.add_argument('--write-config', help='Guardar configuración inicial y salir')
    p.add_argument('--preview', action='store_true', help='Requiere OpenCV con GUI y escritorio')
    p.add_argument('--save', help='Guardar imagen anotada en modo --image')
    args = p.parse_args()
    if args.sbs_left and args.sbs_both:
        p.error('--sbs-left y --sbs-both son excluyentes')
    cfg = json.loads(Path(args.config).read_text()) if args.config else DEFAULT
    if args.write_config:
        Path(args.write_config).write_text(json.dumps(cfg, indent=2)+'\n')
        return
    if not 0 < cfg['min_radius'] < cfg['max_radius']:
        p.error('Radios inválidos')
    for region in cfg['regions'].values():
        x0, y0, x1, y1 = region
        if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
            p.error('Región inválida')
    cap = None
    tracker = Confirmation(cfg)
    sequence = 0
    try:
        if not args.image:
            source = int(args.source) if args.source.isdecimal() else args.source
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                raise RuntimeError('No se pudo abrir la cámara o video')
        while True:
            if args.image:
                frame = cv2.imread(args.image)
                ok = frame is not None
            else:
                ok, frame = cap.read()
            if not ok:
                print(json.dumps({'valid': False, 'reason': 'sin_imagen', 'seq': sequence}), flush=True)
                break
            if args.sbs_both:
                w = frame.shape[1]
                image_left, found_left = detect(frame[:, :w // 2], cfg)
                image_right, found_right = detect(frame[:, w // 2:], cfg)
                found = {name: found_left[name] + found_right[name] for name in cfg['regions']}
                for name in found:
                    found[name].sort(key=lambda c: c['support'], reverse=True)
            else:
                if args.sbs_left:
                    frame = frame[:, :frame.shape[1]//2]
                image_left, found = detect(frame, cfg)
                found_left, image_right, found_right = found, None, None
            now = time.monotonic()
            # Un candidato verde SÍ se reporta en 'candidates' (depuración) pero NUNCA
            # cuenta para arriba/abajo: evita que el servo lo recoja. "1" en arriba/abajo
            # significa candidato confirmado Y recogible, no solo "hay algo redondo".
            pickable = {name: [c for c in found[name] if c['color'] != 'verde']
                        for name in found}
            state = tracker.update(pickable, now)
            packet = {'valid': True, 'seq': sequence, 't_monotonic_ms': round(now*1000),
                      'arriba': state['arriba'], 'abajo': state['abajo'],
                      'candidates': found}
            if args.image:
                packet['mode'] = 'imagen_sin_confirmacion_temporal'
                packet['arriba'] = packet['abajo'] = None
            print(json.dumps(packet), flush=True)
            if args.preview or args.save:
                image_left = annotate(image_left, found_left, state, cfg)
                if args.sbs_both:
                    image_right = annotate(image_right, found_right, state, cfg)
                    image = np.hstack([image_left, image_right])
                else:
                    image = image_left
                if args.save and args.image:
                    if not cv2.imwrite(args.save, image):
                        raise RuntimeError('No se pudo guardar la imagen')
                if args.preview:
                    cv2.imshow('Geometria', image)
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
