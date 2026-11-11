"""Compara una fotografía con su corrección de distorsión mediante el YAML."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from vision import (CALIBRACION, FOTOS, RESULTADOS, guardar_imagen,
                    leer_calibracion, leer_imagen, preparar_interfaz)


def comparar(original, corregida):
    vistas = []
    for imagen, titulo in [(original, "ORIGINAL"), (corregida, "RECTIFICADA")]:
        vista = cv2.copyMakeBorder(imagen, 80, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 35, 50))
        cv2.putText(vista, titulo, (23, 55), cv2.FONT_HERSHEY_SIMPLEX,
                    1.2, (255, 255, 255), 2, cv2.LINE_AA)
        vistas.append(vista)
    return np.hstack(vistas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--foto", type=Path, default=FOTOS / "WIN_20260924_15_09_26_Pro.jpg")
    parser.add_argument("--calibracion", type=Path, default=CALIBRACION)
    parser.add_argument("--salida", type=Path, default=RESULTADOS / "comparacion_original_rectificada.jpg")
    parser.add_argument("--sin-ventana", action="store_true", help="Guarda el resultado sin abrir la interfaz.")
    args = parser.parse_args()
    try:
        K, D, tamano = leer_calibracion(args.calibracion)
        original = leer_imagen(args.foto, tamano)
        corregida = cv2.undistort(original, K, D)
        comparacion = comparar(original, corregida)
        guardar_imagen(args.salida, comparacion)
        print(f"Foto: {args.foto.name}\nComparación: {args.salida}")
        if not args.sin_ventana:
            preparar_interfaz()
            cv2.namedWindow("Original y rectificada", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Original y rectificada", 1280, 360)
            cv2.imshow("Original y rectificada", comparacion)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    except (ValueError, cv2.error) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
