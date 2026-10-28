"""
Prueba RGB-xpgm: clasifica el color de una pelota por distancia a centroides fijos
en espacio RGB crudo (el metodo "picos de color" que se sugirio al inicio), usando
el mismo set de colores que detector_pelotas (rojo, naranja, amarillo, negro, azul;
verde solo para poder excluirlo, igual que el detector real).

Objetivo: tomar un parche real de una foto de banco y ver si la clasificacion se
mantiene estable cuando se oscurece/ilumina artificialmente (sombra/reflejo
simulados), para comparar contra HSV-xpgm.py con el mismo parche.

Esto NO modifica ni importa nada de vision_side_quest_xpgm: solo LEE una imagen de
ahi como referencia.
"""
import argparse
import sys

import numpy as np
import cv2

# Centroides en BGR (OpenCV carga asi, no RGB). Rojo y verde aproximan los valores
# de bench reales documentados en detector_pelotas/README.md; naranja/amarillo/
# negro/azul son PLACEHOLDER sin calibrar con pelotas reales de esos colores.
REFERENCE_BGR = {
    "rojo": (40, 40, 220),
    "naranja": (30, 110, 230),
    "amarillo": (40, 210, 220),
    "verde": (60, 160, 60),
    "azul": (200, 90, 40),
    "negro": (20, 20, 20),
}

LIGHT_FACTORS = (0.3, 0.5, 0.7, 1.0, 1.3, 1.6)


def classify_rgb(bgr_pixel):
    best_color, best_dist = None, float("inf")
    for color, ref in REFERENCE_BGR.items():
        dist = float(np.linalg.norm(np.array(bgr_pixel, dtype=float) - np.array(ref, dtype=float)))
        if dist < best_dist:
            best_dist, best_color = dist, color
    return best_color, best_dist


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
    patch = img[max(0, y - half):y + half, max(0, x - half):x + half]
    base_bgr = patch.reshape(-1, 3).mean(axis=0)

    print(f"Imagen: {args.image} ({w}x{h})")
    print(f"Parche muestreado en ({x},{y}), {args.patch}x{args.patch}px -> BGR promedio = {base_bgr.round(1)}")
    print()
    print(f"{'factor luz':>10} | {'BGR simulado':>22} | {'clasificado':>10} | distancia")
    print("-" * 62)
    for factor in LIGHT_FACTORS:
        sim = np.clip(base_bgr * factor, 0, 255)
        color, dist = classify_rgb(sim)
        bgr_str = f"({sim[0]:.0f},{sim[1]:.0f},{sim[2]:.0f})"
        print(f"{factor:>10.1f} | {bgr_str:>22} | {color:>10} | {dist:.1f}")


if __name__ == "__main__":
    main()
