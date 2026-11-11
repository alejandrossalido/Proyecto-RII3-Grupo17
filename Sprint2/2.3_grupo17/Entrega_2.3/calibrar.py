"""Calibra la cámara y registra las fotografías y errores del ajuste final."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from vision import (CALIBRACION, FOTOS, PATRON, RESULTADOS, buscar_esquinas,
                    error_rms, leer_imagen, puntos_tablero)


def ajustar(detectadas, tamano, umbral=3.0, minimo=8):
    """Filtra vistas y recalcula siempre sobre la lista final de fotografías."""
    usadas = list(detectadas)
    if len(usadas) < minimo:
        raise ValueError(f"Se necesitan al menos {minimo} fotos con el patrón completo.")
    objeto = puntos_tablero()
    while True:
        rms, K, D, rotaciones, traslaciones = cv2.calibrateCamera(
            [objeto.copy() for _ in usadas], [p for _, p in usadas],
            tamano, None, None, flags=cv2.CALIB_FIX_K3)
        errores = []
        for (_, esquinas), rot, tras in zip(usadas, rotaciones, traslaciones):
            proyectadas, _ = cv2.projectPoints(objeto, rot, tras, K, D)
            errores.append(error_rms(esquinas, proyectadas))
        buenas = [par for par, error in zip(usadas, errores) if error <= umbral]
        if len(buenas) == len(usadas):
            break
        if len(buenas) < minimo:
            raise ValueError("El filtrado deja muy pocas fotos. Revisa la calidad del tablero.")
        usadas = buenas
    if not np.isfinite(rms) or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError("La calibración no produjo parámetros finitos.")
    if not (0 < K[0, 2] < tamano[0] and 0 < K[1, 2] < tamano[1]):
        raise ValueError("Centro óptico fuera de la imagen. Revisa las fotos y usa el entorno de requirements.txt. No se ha sobrescrito el YAML.")
    return float(rms), K, D, usadas, errores


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fotos", type=Path, default=FOTOS)
    parser.add_argument("--salida", type=Path, default=CALIBRACION)
    parser.add_argument("--informe", type=Path, default=RESULTADOS / "calibracion.json")
    args = parser.parse_args()
    fotos = sorted(args.fotos.glob("*.jpg"))
    if not fotos:
        parser.error(f"No hay fotos JPG en {args.fotos}")
    try:
        detectadas, tamano = [], None
        for foto in fotos:
            imagen = leer_imagen(foto, tamano)
            tamano = (imagen.shape[1], imagen.shape[0])
            esquinas = buscar_esquinas(cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY))
            if esquinas is not None:
                detectadas.append((foto.name, esquinas))
            print(f"{foto.name}: {'patrón completo' if esquinas is not None else 'sin patrón completo'}", flush=True)
        rms, K, D, usadas, errores = ajustar(detectadas, tamano)
        args.salida.parent.mkdir(parents=True, exist_ok=True)
        archivo = cv2.FileStorage(str(args.salida), cv2.FILE_STORAGE_WRITE)
        if not archivo.isOpened():
            raise ValueError(f"No se puede escribir {args.salida}")
        for clave, valor in {"image_width": tamano[0], "image_height": tamano[1],
                             "camera_matrix": K, "distortion_coefficients": D,
                             "reprojection_rms": rms, "pattern_inner_corners_x": PATRON[0],
                             "pattern_inner_corners_y": PATRON[1], "square_size": 1.0,
                             "usable_image_count": len(usadas)}.items():
            archivo.write(clave, valor)
        archivo.release()
        nombres_usadas = {nombre for nombre, _ in usadas}
        nombres_detectadas = {nombre for nombre, _ in detectadas}
        informe = {"opencv": cv2.__version__, "resolucion": list(tamano),
                   "fotos_totales": len(fotos), "patron_detectado": len(detectadas),
                   "fotos_usadas": len(usadas), "rms_reproyeccion_px": rms,
                   "umbral_filtrado_px": 3.0, "camera_matrix": K.tolist(),
                   "distortion_coefficients": D.reshape(-1).tolist(),
                   "usadas": [{"foto": nombre, "error_rms_px": error}
                              for (nombre, _), error in zip(usadas, errores)],
                   "descartadas_por_error": sorted(nombres_detectadas - nombres_usadas),
                   "sin_patron": [p.name for p in fotos if p.name not in nombres_detectadas]}
        args.informe.parent.mkdir(parents=True, exist_ok=True)
        args.informe.write_text(json.dumps(informe, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\nFotos: {len(fotos)}; patrón: {len(detectadas)}; usadas: {len(usadas)}")
        print(f"RMS: {rms:.6f} px. Calibración: {args.salida}")
    except (ValueError, cv2.error) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
