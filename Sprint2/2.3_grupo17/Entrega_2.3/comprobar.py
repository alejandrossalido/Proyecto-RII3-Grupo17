"""Mide la rectitud del patrón en todas las fotos antes y después de corregir."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from vision import (CALIBRACION, FOTOS, RESULTADOS, PATRON, buscar_esquinas,
                    guardar_imagen, leer_calibracion, leer_imagen)
from rectificar import comparar


def rectitud(puntos):
    """RMS perpendicular a la mejor recta de cada fila y columna, en píxeles."""
    a = np.asarray(puntos, dtype=np.float32).reshape(PATRON[1], PATRON[0], 2)
    distancias = []
    for linea in list(a) + list(a.transpose(1, 0, 2)):
        vx, vy, x, y = cv2.fitLine(linea, cv2.DIST_L2, 0, .01, .01).ravel()
        distancias.extend(((linea[:, 0] - x) * vy - (linea[:, 1] - y) * vx).tolist())
    return float(np.sqrt(np.mean(np.square(distancias))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fotos", type=Path, default=FOTOS)
    parser.add_argument("--calibracion", type=Path, default=CALIBRACION)
    parser.add_argument("--salida", type=Path, default=RESULTADOS)
    args = parser.parse_args()
    try:
        K, D, tamano = leer_calibracion(args.calibracion)
        filas = []
        fotos = sorted(args.fotos.glob("*.jpg"))
        if not fotos:
            raise ValueError("No hay fotos JPG para comprobar.")
        ejemplos = {"WIN_20260924_15_09_04_Pro.jpg": "ejemplo_original",
                    "WIN_20260924_15_09_26_Pro.jpg": "ejemplo_mejora",
                    "WIN_20260924_15_09_35_Pro.jpg": "ejemplo_empeora"}
        for foto in fotos:
            imagen = leer_imagen(foto, tamano)
            esquinas = buscar_esquinas(cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY))
            if esquinas is None:
                continue
            corregidas = cv2.undistortPoints(esquinas, K, D, P=K).reshape(-1, 2)
            antes, despues = rectitud(esquinas), rectitud(corregidas)
            filas.append({"foto": foto.name, "rectitud_original_px": antes,
                          "rectitud_corregida_px": despues, "mejora": despues < antes})
            print(f"{foto.name}: {antes:.3f} a {despues:.3f} px", flush=True)
            if foto.name in ejemplos:
                und = cv2.undistort(imagen, K, D)
                puntos = np.vstack((esquinas.reshape(-1, 2), corregidas))
                x0, y0 = np.maximum(np.floor(puntos.min(axis=0)).astype(int) - 35, 0)
                x1, y1 = np.minimum(np.ceil(puntos.max(axis=0)).astype(int) + 35, tamano)
                if x1 > x0 and y1 > y0:
                    guardar_imagen(args.salida / f"{ejemplos[foto.name]}_detalle.jpg",
                                   comparar(imagen[y0:y1, x0:x1], und[y0:y1, x0:x1]))
        if not filas:
            raise ValueError("No se ha encontrado ningún patrón completo.")
        args.salida.mkdir(parents=True, exist_ok=True)
        with (args.salida / "mediciones_rectitud.csv").open("w", newline="", encoding="utf-8") as archivo:
            escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
            escritor.writeheader()
            escritor.writerows(filas)
        resumen = {"opencv": cv2.__version__, "resolucion": list(tamano),
                   "fotos_totales": len(fotos), "fotos_con_patron": len(filas),
                   "mejoran": sum(f["mejora"] for f in filas),
                   "mediana_original_px": float(np.median([f["rectitud_original_px"] for f in filas])),
                   "mediana_corregida_px": float(np.median([f["rectitud_corregida_px"] for f in filas])),
                   "ejemplos": [f for f in filas if f["foto"] in ejemplos],
                   "metodo": "Distancia RMS perpendicular de esquinas a rectas por filas y columnas. Mismas correspondencias con undistortPoints(P=K).",
                   "limitaciones": ["Fotos del mismo conjunto de calibración, sin validación independiente.",
                                    "El patrón presenta ondulaciones. No permite aislar distorsión óptica y curvatura del papel.",
                                    "La medida en píxeles también depende de la escala. No es el RMS de reproyección."]}
        (args.salida / "comprobacion_rectitud.json").write_text(
            json.dumps(resumen, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps(resumen, indent=2, ensure_ascii=False))
    except (ValueError, cv2.error) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
