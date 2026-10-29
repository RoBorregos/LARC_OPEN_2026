"""
Prueba HSV-xpgm: clasifica el color de una pelota por Hue (+ saturacion/valor
minimos), mismo set de colores y mismo parche que RGB-xpgm.py, para comparar
directamente los dos metodos bajo el mismo cambio de luz simulado.

Rangos de rojo y verde copiados de
vision_side_quest_xpgm/detector_pelotas/config.json (ya calibrados en banco con
pelotas reales). naranja/amarillo/azul/negro son PLACEHOLDER: faltan calibrar con
pelotas reales de esos colores antes de competencia.

Esto NO modifica ni importa nada de vision_side_quest_xpgm: solo LEE una imagen de
ahi como referencia.
"""
import argparse
import sys

import numpy as np
import cv2

# Rangos de Hue en escala OpenCV (0-179). Rojo cruza el 0 del circulo de matiz,
# por eso tiene dos rangos, igual que en el detector real.
HUE_RANGES = {
    "rojo": [(0, 10), (170, 179)],
    "naranja": [(11, 25)],
    "amarillo": [(26, 34)],
    "verde": [(40, 100)],
    "azul": [(100, 130)],
}
MIN_SATURATION = 90  # igual que min_ball_saturation en config.json
MIN_VALUE = 15  # igual que min_ball_value en config.json
BLACK_MAX_VALUE = 50  # negro no se define por Hue, se define por brillo bajo

LIGHT_FACTORS = (0.3, 0.5, 0.7, 1.0, 1.3, 1.6)


def classify_hsv(h, s, v):
    if v < BLACK_MAX_VALUE:
        return "negro"
    if s < MIN_SATURATION or v < MIN_VALUE:
        return None  # ni color de pelota ni negro: fondo/blanco/sombra ambigua
    for color, ranges in HUE_RANGES.items():
        for lo, hi in ranges:
            if lo <= h <= hi:
                return color
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--image",
        default="../vision_side_quest_xpgm/detector_pelotas/zed_now.png",
        help="Imagen de referencia (default: foto real de banco ya existente)",
    )
    ap.add_argument("--x", type=int, default=None, help="Columna del parche a muestrear")
    ap.add_argument("--y", type=int, default=None, help="Fila del parche a muestrear")
    ap.add_argument("--patch", type=int, default=15, help="Lado del parche cuadrado a promediar (px)")
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        sys.exit(f"No se pudo leer {args.image}")

    h, w = img.shape[:2]
    x = args.x if args.x is not None else w // 2
    y = args.y if args.y is not None else h // 2
    half = max(1, args.patch // 2)
    patch_bgr = img[max(0, y - half):y + half, max(0, x - half):x + half]
    base_bgr = patch_bgr.reshape(-1, 3).mean(axis=0)

    print(f"Imagen: {args.image} ({w}x{h})")
    print(f"Parche muestreado en ({x},{y}), {args.patch}x{args.patch}px")
    print()
    print(f"{'factor luz':>10} | {'H':>5} {'S':>5} {'V':>5} | clasificado")
    print("-" * 46)
    for factor in LIGHT_FACTORS:
        sim_bgr = np.clip(base_bgr * factor, 0, 255).astype(np.uint8).reshape(1, 1, 3)
        hsv_pixel = cv2.cvtColor(sim_bgr, cv2.COLOR_BGR2HSV)[0, 0]
        color = classify_hsv(int(hsv_pixel[0]), int(hsv_pixel[1]), int(hsv_pixel[2]))
        print(f"{factor:>10.1f} | {hsv_pixel[0]:>5} {hsv_pixel[1]:>5} {hsv_pixel[2]:>5} | {color}")


if __name__ == "__main__":
    main()
