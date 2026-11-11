"""Detecta identificadores ArUco y dibuja sus ejes sobre fotografías del robot."""
import argparse
import itertools
import json
from pathlib import Path
import cv2
from vision import (CALIBRACION, FOTOS, RESULTADOS, crear_detector, detectar_marcadores,
                    guardar_imagen, leer_calibracion, leer_imagen, preparar_interfaz)

EJEMPLO = "WIN_20260924_15_12_32_Pro.jpg"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fotos", type=Path, default=FOTOS)
    parser.add_argument("--calibracion", type=Path, default=CALIBRACION)
    parser.add_argument("--salida", type=Path, default=RESULTADOS)
    parser.add_argument("--sin-ventana", action="store_true", help="Procesa todas las fotos y guarda un informe.")
    args = parser.parse_args()
    fotos = sorted(args.fotos.glob("*.jpg"))
    if not fotos:
        parser.error(f"No hay fotos JPG en {args.fotos}")
    try:
        K, D, tamano = leer_calibracion(args.calibracion)
        detector = crear_detector()
        indice = next((i for i, foto in enumerate(fotos) if foto.name == EJEMPLO), 0)
        registros = []
        tiene_ejemplo = any(p.name == EJEMPLO for p in fotos)
        if not args.sin_ventana:
            preparar_interfaz()
            cv2.namedWindow("ArUco - pasar fotos", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ArUco - pasar fotos", 1280, 720)
        for numero in range(len(fotos)) if args.sin_ventana else itertools.count():
            if args.sin_ventana:
                indice = numero
            foto = fotos[indice]
            imagen = leer_imagen(foto, tamano)
            salida, marcadores = detectar_marcadores(imagen, detector, K, D)
            ids = [m["id"] for m in marcadores]
            if args.sin_ventana:
                registros.append({"foto": foto.name, "ids": ids, "marcadores": marcadores})
            if foto.name == EJEMPLO or (not tiene_ejemplo and indice == 0):
                guardar_imagen(args.salida / "aruco_ids_y_ejes.jpg", salida)
            print(f"Foto {indice + 1}/{len(fotos)}: {foto.name}: IDs {ids}", flush=True)
            if args.sin_ventana:
                continue
            cv2.rectangle(salida, (0, 0), (salida.shape[1], 90), (20, 35, 50), -1)
            cv2.putText(salida, f"Foto {indice + 1}/{len(fotos)} - IDs: {ids}",
                        (22, 38), cv2.FONT_HERSHEY_SIMPLEX, .85, (255, 255, 255), 2)
            cv2.putText(salida, "ESPACIO/N: siguiente    P: anterior    Q: salir", (22, 76),
                        cv2.FONT_HERSHEY_SIMPLEX, .75, (220, 230, 240), 2)
            cv2.imshow("ArUco - pasar fotos", salida)
            tecla = cv2.waitKeyEx(0)
            if tecla in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty("ArUco - pasar fotos", cv2.WND_PROP_VISIBLE) < 1:
                break
            if tecla in (ord("p"), ord("P")):
                indice = (indice - 1) % len(fotos)
            elif tecla in (ord("n"), ord("N"), 32):
                indice = (indice + 1) % len(fotos)
        if args.sin_ventana:
            args.salida.mkdir(parents=True, exist_ok=True)
            informe = {"opencv": cv2.__version__, "diccionario": "DICT_4X4_50", "lado_marcador": 1.0,
                       "unidad_traslacion": "lados del marcador, no metros",
                       "fotos_procesadas": len(registros),
                       "fotos_con_aruco": sum(bool(r["ids"]) for r in registros),
                       "ids_distintos": sorted({i for r in registros for i in r["ids"]}),
                       "resultados": registros}
            (args.salida / "deteccion_aruco.json").write_text(
                json.dumps(informe, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        else:
            cv2.destroyAllWindows()
    except (ValueError, cv2.error) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
