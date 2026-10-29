#!/usr/bin/env python3
"""Copia AISLADA (no importa separator_vision.py) de la logica de
clasificacion del sorter, mas un clasificador RGB nuevo para comparar contra
la HSV real -- mismo patron que detector-xpgm.py con detector.py.

DEFAULT_CFG, _pct(), masks_for() y classify() son copia textual de
vision_orin/separator/separator_vision.py (metodo HSV, el que ya corre en
produccion). classify_rgb_mean() es nuevo: no existe en el pipeline real,
responde la pregunta de si vale la pena cambiar a RGB para "evitar
calibracion". Centroides de referencia iguales a RGB-xpgm.py, para que la
comparacion use los mismos numeros en todo xpgm-testVision.

Este archivo NO importa nada de separator/: si ese pipeline cambia, este no
se entera y no se rompe -- y viceversa, nada de aqui afecta al real.
"""

DEFAULT_CFG = dict(
    roi_x=551, roi_y=208, roi_w=197, roi_h=251,
    warm_h_lo=0,   warm_h_hi=35,
    warm_s_min=80, warm_v_min=80,
    cool_h_lo=95,  cool_h_hi=135,
    cool_s_min=60, cool_v_min=40,
    green_h_lo=36, green_h_hi=92,
    green_s_min=60, green_v_min=40,
    black_v_max=50,
    white_s_max=40,
    white_v_min=180,
    warm_frac=5,
    cool_frac=5,
    green_frac=5,
    white_frac=60,
    unknown_to_mature=0,
    unknown_cover_min=35,
    morph_k=5,
    morph_iter=1,
)


def _pct(mask, roi_pixels):
    import numpy as np
    return np.count_nonzero(mask) / roi_pixels * 100


def masks_for(roi_hsv, cfg):
    import cv2
    import numpy as np
    warm = cv2.inRange(roi_hsv,
        np.array([cfg['warm_h_lo'], cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
        np.array([cfg['warm_h_hi'], 255, 255], np.uint8))
    if cfg['warm_h_lo'] < 10:
        warm = cv2.bitwise_or(warm, cv2.inRange(roi_hsv,
            np.array([160, cfg['warm_s_min'], cfg['warm_v_min']], np.uint8),
            np.array([180, 255, 255], np.uint8)))

    green = cv2.inRange(roi_hsv,
        np.array([cfg['green_h_lo'], cfg['green_s_min'], cfg['green_v_min']], np.uint8),
        np.array([cfg['green_h_hi'], 255, 255], np.uint8))

    cool = cv2.inRange(roi_hsv,
        np.array([cfg['cool_h_lo'], cfg['cool_s_min'], cfg['cool_v_min']], np.uint8),
        np.array([cfg['cool_h_hi'], 255, 255], np.uint8))
    blk  = (roi_hsv[:, :, 2] < cfg['black_v_max']).astype(np.uint8) * 255
    cool = cv2.bitwise_or(cool, blk)

    white = ((roi_hsv[:, :, 1] < cfg['white_s_max']) &
             (roi_hsv[:, :, 2] > cfg['white_v_min'])).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg['morph_k'], cfg['morph_k']))
    n = cfg['morph_iter']
    out = {}
    for name, mask in (("warm", warm), ("cool", cool),
                       ("green", green), ("white", white)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=n)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=n)
        out[name] = mask
    return out


def classify(pct, cfg):
    if pct['white'] >= cfg['white_frac']:
        return False, False, True, "no ball"

    green_hit = pct['green'] >= cfg['green_frac']
    warm_hit  = pct['warm']  >= cfg['warm_frac']
    cool_hit  = pct['cool']  >= cfg['cool_frac']
    if green_hit:
        return True, False, False, "green -> MATURE"

    if warm_hit or cool_hit:
        if warm_hit and cool_hit:
            return True, True, False, "ambiguous (warm+cool)"
        return warm_hit, cool_hit, False, (
            "warm -> MATURE" if warm_hit else "cool -> IMMATURE")
    if cfg.get('unknown_to_mature', 0):
        if (100.0 - pct['white']) >= cfg['unknown_cover_min']:
            return True, False, False, "unknown -> MATURE (fallback)"

    return False, False, False, "unknown -> NEUTRAL (background?)"


# ---------------------------------------------------------------------------
# Clasificador RGB nuevo (no existe en separator_vision.py) -- mismos
# centroides BGR de referencia que RGB-xpgm.py, para responder "nos ahorramos
# calibracion cambiando a RGB?" con el mismo ROI que usa el sorter real.
# ---------------------------------------------------------------------------
REFERENCE_BGR = {
    "rojo": (40, 40, 220),
    "naranja": (30, 110, 230),
    "amarillo": (40, 210, 220),
    "verde": (60, 160, 60),
    "azul": (200, 90, 40),
    "negro": (20, 20, 20),
}

# Mismo mapeo warm/green->MADURA, cool/negro->INMADURA que classify() de
# arriba (HSV), para que las dos salidas sean comparables cuadro a cuadro.
RGB_BIN = {
    "rojo": "MADURA", "naranja": "MADURA", "amarillo": "MADURA", "verde": "MADURA",
    "azul": "INMADURA", "negro": "INMADURA",
}


def classify_rgb_mean(mean_bgr):
    """Centroide mas cercano en BGR crudo sobre el promedio del ROI.
    Devuelve (color, bin, distancia)."""
    import numpy as np
    best_color, best_dist = None, float("inf")
    for color, ref in REFERENCE_BGR.items():
        dist = float(np.linalg.norm(np.array(mean_bgr, dtype=float) - np.array(ref, dtype=float)))
        if dist < best_dist:
            best_dist, best_color = dist, color
    return best_color, RGB_BIN[best_color], best_dist
