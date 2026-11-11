"""Cámara: calibración, rectificación y ArUco desde un único comando."""

import argparse
from pathlib import Path

import cv2
import numpy as np

CARPETA = Path(__file__).resolve().parent
FOTOS = CARPETA / 'Fotos_Sprint2_2.3'
CALIBRACION = CARPETA / 'calibracion_jetbot.yml'


# Lee la matriz de camara y los coeficientes de distorsion guardados.
def cargar_calibracion():
    archivo = cv2.FileStorage(str(CALIBRACION), cv2.FILE_STORAGE_READ)
    if not archivo.isOpened():
        raise SystemExit('No se encuentra calibracion_jetbot.yml')
    matriz = archivo.getNode('camera_matrix').mat()
    distorsion = archivo.getNode('distortion_coefficients').mat()
    archivo.release()
    return matriz, distorsion


# Calcula y guarda la calibracion a partir de las fotos del tablero.
def calibrar(_foto=None):
    patron = (13, 9)
    tablero = np.zeros((13*9, 3), np.float32)
    tablero[:, :2] = np.mgrid[0:13, 0:9].T.reshape(-1, 2)
    puntos_3d, puntos_2d = [], []
    tamano = None
    for foto in sorted(FOTOS.glob('*.jpg')):
        imagen = cv2.imread(str(foto), cv2.IMREAD_GRAYSCALE)
        if imagen is None:
            continue
        pequena = cv2.resize(imagen, None, fx=0.5, fy=0.5)
        encontrado, esquinas = cv2.findChessboardCornersSB(
            pequena, patron, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if encontrado:
            esquinas *= 2
            cv2.cornerSubPix(imagen, esquinas, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
        else:
            encontrado, esquinas = cv2.findChessboardCornersSB(
                imagen, patron, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if encontrado:
            puntos_3d.append(tablero)
            puntos_2d.append(esquinas)
            tamano = (imagen.shape[1], imagen.shape[0])
            print('Tablero encontrado:', foto.name)
    if len(puntos_3d) < 8:
        raise SystemExit('Se necesitan al menos 8 fotos con el tablero completo.')
    error, matriz, distorsion, _, _ = cv2.calibrateCamera(
        puntos_3d, puntos_2d, tamano, None, None, flags=cv2.CALIB_FIX_K3)
    archivo = cv2.FileStorage(str(CALIBRACION), cv2.FILE_STORAGE_WRITE)
    for nombre, valor in dict(image_width=tamano[0], image_height=tamano[1],
            camera_matrix=matriz, distortion_coefficients=distorsion,
            reprojection_rms=error, usable_image_count=len(puntos_3d)).items():
        archivo.write(nombre, valor)
    archivo.release()
    print(f'Calibración guardada. Fotos: {len(puntos_3d)}; RMS: {error:.3f} píxeles')


# Rectifica una foto o dibuja los marcadores ArUco y sus ejes.
def procesar_foto(accion, foto):
    matriz, distorsion = cargar_calibracion()
    imagen = cv2.imread(str(foto))
    if imagen is None:
        raise SystemExit(f'No se puede abrir {foto}')
    if accion == 'rectificar':
        imagen = np.hstack((imagen, cv2.undistort(imagen, matriz, distorsion)))
        nombre = 'comparacion_original_rectificada.jpg'
    else:
        diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parametros = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
                      else cv2.aruco.DetectorParameters_create())
        esquinas, ids, _ = cv2.aruco.detectMarkers(imagen, diccionario, parameters=parametros)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(imagen, esquinas, ids)
            giros, posiciones, _ = cv2.aruco.estimatePoseSingleMarkers(
                esquinas, 1.0, matriz, distorsion)
            for giro, posicion in zip(giros, posiciones):
                cv2.drawFrameAxes(imagen, matriz, distorsion, giro, posicion, 0.5)
        print('IDs encontrados:', [] if ids is None else ids.flatten().tolist())
        nombre = 'aruco_ids_y_ejes.jpg'
    salida = CARPETA / 'resultados' / nombre
    salida.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(salida), imagen)
    print('Resultado guardado en:', salida)


# Lee los argumentos y ejecuta la operacion de camara elegida.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('accion', choices=('calibrar', 'rectificar', 'aruco'))
    parser.add_argument('foto', nargs='?', type=Path)
    args = parser.parse_args()
    if args.accion == 'calibrar':
        calibrar()
    else:
        defecto = 'WIN_20260924_15_09_04_Pro.jpg' if args.accion == 'rectificar' else 'WIN_20260924_15_12_32_Pro.jpg'
        procesar_foto(args.accion, args.foto or FOTOS / defecto)


if __name__ == '__main__':
    main()
