"""Funciones comunes de calibración y detección para el entorno OpenCV fijado."""
from pathlib import Path
import os
import sys
import cv2
import numpy as np

CARPETA = Path(__file__).resolve().parent
FOTOS = CARPETA / "Fotos_Sprint2_2.3"
CALIBRACION = CARPETA / "calibracion_jetbot.yml"
RESULTADOS = CARPETA / "resultados"
PATRON = (13, 9)  # Esquinas interiores, no número de casillas.
cv2.setNumThreads(4)


def preparar_interfaz():
    """Usa XWayland si está disponible y fuentes existentes para Qt en Linux."""
    if sys.platform != "linux":
        return
    if os.environ.get("DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    if not Path(os.environ.get("QT_QPA_FONTDIR", "/ruta_inexistente")).is_dir():
        for ruta in ("/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/TTF",
                     "/usr/share/fonts/liberation", "/usr/share/fonts/noto"):
            if Path(ruta).is_dir() and any(Path(ruta).glob("*.ttf")):
                os.environ["QT_QPA_FONTDIR"] = ruta
                break


def buscar_esquinas(gris):
    """Devuelve siempre float32 con forma (117, 1, 2), también en OpenCV 5."""
    if gris is None:
        raise ValueError("No se ha podido leer la imagen.")
    pequena = cv2.resize(gris, None, fx=0.5, fy=0.5)
    ok, esquinas = cv2.findChessboardCornersSB(
        pequena, PATRON, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        esquinas = np.ascontiguousarray(esquinas.reshape(-1, 1, 2), np.float32) * 2
        cv2.cornerSubPix(gris, esquinas, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    else:
        ok, esquinas = cv2.findChessboardCornersSB(
            gris, PATRON, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return np.ascontiguousarray(esquinas.reshape(-1, 1, 2), np.float32) if ok else None


def puntos_tablero():
    puntos = np.zeros((PATRON[0] * PATRON[1], 3), np.float32)
    puntos[:, :2] = np.mgrid[0:PATRON[0], 0:PATRON[1]].T.reshape(-1, 2)
    return puntos


def error_rms(observadas, proyectadas):
    """Distancia RMS punto a punto en píxeles, independiente de la forma OpenCV."""
    a = np.asarray(observadas, dtype=np.float64).reshape(-1, 2)
    b = np.asarray(proyectadas, dtype=np.float64).reshape(-1, 2)
    if a.shape != b.shape or not a.size:
        raise ValueError("Las correspondencias deben tener el mismo número de puntos.")
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


def leer_calibracion(ruta=CALIBRACION):
    archivo = cv2.FileStorage(str(ruta), cv2.FILE_STORAGE_READ)
    if not archivo.isOpened():
        raise ValueError(f"No se puede abrir {ruta}. Ejecuta primero calibrar.py.")
    try:
        K = archivo.getNode("camera_matrix").mat()
        D = archivo.getNode("distortion_coefficients").mat()
        tamano = (int(archivo.getNode("image_width").real()),
                  int(archivo.getNode("image_height").real()))
    finally:
        archivo.release()
    if (K is None or D is None or K.shape != (3, 3) or D.size != 5
            or not np.isfinite(K).all() or not np.isfinite(D).all()
            or min(tamano) <= 0 or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise ValueError("El fichero de calibración contiene parámetros inválidos.")
    return K, D, tamano


def leer_imagen(ruta, tamano=None):
    imagen = cv2.imread(str(ruta))
    if imagen is None:
        raise ValueError(f"No se puede leer {ruta}")
    if tamano is not None and (imagen.shape[1], imagen.shape[0]) != tamano:
        raise ValueError(f"Resolución incompatible en {Path(ruta).name}: se espera {tamano}.")
    return imagen


def guardar_imagen(ruta, imagen):
    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(ruta), imagen):
        raise ValueError(f"No se pudo guardar {ruta}")


def crear_detector():
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise ValueError("Instala requirements.txt (OpenCV >= 4.8 con ArUco).")
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.ArucoDetector(diccionario, cv2.aruco.DetectorParameters())


def puntos_marcador(lado=1.0):
    if not np.isfinite(lado) or lado <= 0:
        raise ValueError("El lado del marcador debe ser positivo.")
    medio = lado / 2
    # Orden requerido por SOLVEPNP_IPPE_SQUARE y por las esquinas de ArUco.
    return np.array([[-medio, medio, 0], [medio, medio, 0],
                     [medio, -medio, 0], [-medio, -medio, 0]], np.float32)


def detectar_marcadores(imagen, detector, K, D, lado=1.0):
    esquinas, ids, _ = detector.detectMarkers(imagen)
    salida = imagen.copy()
    marcadores = []
    if ids is None:
        return salida, marcadores
    cv2.aruco.drawDetectedMarkers(salida, esquinas)
    objeto = puntos_marcador(lado)
    for esquina, identificador in zip(esquinas, ids.flatten()):
        puntos = np.ascontiguousarray(esquina.reshape(4, 2), np.float32)
        ok, rotacion, traslacion = cv2.solvePnP(
            objeto, puntos, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if (not ok or not np.isfinite(rotacion).all() or not np.isfinite(traslacion).all()
                or traslacion[2, 0] <= 0):
            raise ValueError(f"No se pudo estimar una pose válida para el ArUco {identificador}.")
        cv2.drawFrameAxes(salida, K, D, rotacion, traslacion, lado * .55, 3)
        proyectadas, _ = cv2.projectPoints(objeto, rotacion, traslacion, K, D)
        marcadores.append({"id": int(identificador), "rvec": rotacion.reshape(3).tolist(),
                           "tvec": traslacion.reshape(3).tolist(),
                           "error_reproyeccion_px": error_rms(puntos, proyectadas)})
        x, y = np.floor(puntos.min(axis=0)).astype(int)
        x = min(max(int(x), 5), salida.shape[1] - 125)
        y = max(int(y) - 45, 5)
        cv2.rectangle(salida, (x, y), (x + 120, y + 38), (20, 35, 50), -1)
        cv2.putText(salida, f"ID {identificador}", (x + 7, y + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .9, (255, 255, 255), 2, cv2.LINE_AA)
    return salida, marcadores
