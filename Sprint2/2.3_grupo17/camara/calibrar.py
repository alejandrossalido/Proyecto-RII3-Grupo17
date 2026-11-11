from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np

# 1. Archivos y patrón: contamos esquinas interiores, no casillas.
carpeta = Path(__file__).resolve().parent
archivo_zip = carpeta.parent / "WIN_20260924_15_09_04_Pro.zip"
salida = carpeta / "resultados_camara_normal"
patron = (13, 9)
cv2.setNumThreads(4)

# 2. Posiciones del ajedrez plano: una casilla equivale a una unidad.
puntos_3d = np.zeros((patron[0] * patron[1], 3), np.float32)
puntos_3d[:, :2] = np.mgrid[:patron[0], :patron[1]].T.reshape(-1, 2)
puntos_reales, puntos_fotos = [], []
tamano = None

# 3. Buscamos las esquinas en cada foto del ZIP.
with ZipFile(archivo_zip) as fotos:
    nombres = sorted(n for n in fotos.namelist() if n.lower().endswith((".jpg", ".jpeg", ".png")))
    for nombre in nombres:
        imagen = cv2.imdecode(np.frombuffer(fotos.read(nombre), np.uint8), cv2.IMREAD_COLOR)
        if imagen is None:
            continue
        gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        if tamano is None:
            tamano = gris.shape[::-1]
        if gris.shape[::-1] != tamano:
            raise SystemExit("Las fotos deben tener la misma resolución.")
        encontrado, esquinas = cv2.findChessboardCornersSB(gris, patron)
        print(f"{nombre}: {'OK' if encontrado else 'descartada'}", flush=True)
        if encontrado:
            puntos_reales.append(puntos_3d)
            puntos_fotos.append(esquinas)
            original = imagen

if len(puntos_fotos) < 10:
    raise SystemExit("No hay suficientes fotos válidas: recoge al menos 10 vistas del patrón.")

# 4. OpenCV calcula la matriz de cámara y los coeficientes de distorsión.
error, matriz, distorsion, _, _ = cv2.calibrateCamera(
    puntos_reales, puntos_fotos, tamano, None, None
)
if not np.isfinite(error) or not np.isfinite(matriz).all() or not np.isfinite(distorsion).all():
    raise SystemExit("El cálculo no ha producido parámetros válidos.")

# 5. Guardamos los parámetros para reutilizarlos con esta misma cámara.
salida.mkdir(parents=True, exist_ok=True)
yml = cv2.FileStorage(str(salida / "calibracion.yml"), cv2.FILE_STORAGE_WRITE)
if not yml.isOpened():
    raise SystemExit("No se puede guardar calibracion.yml.")
for clave, valor in {
    "camera_name": "camara_normal_de_las_fotos",
    "image_width": tamano[0], "image_height": tamano[1],
    "camera_matrix": matriz, "distortion_coefficients": distorsion,
    "rms_reprojection_error_px": error, "used_images": len(puntos_fotos),
    "validation_status": "experimental_pendiente_de_validacion",
}.items():
    yml.write(clave, valor)
yml.release()

# 6. Corregimos la última foto válida y la comparamos con la original.
corregida = cv2.undistort(original, matriz, distorsion)
comparacion = np.hstack((original, corregida))
for texto, x in (("ORIGINAL", 25), ("CORREGIDA", tamano[0] + 25)):
    cv2.putText(comparacion, texto, (x, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 3)
for nombre, imagen in (("original.jpg", original), ("corregida.jpg", corregida), ("comparacion.jpg", comparacion)):
    if not cv2.imwrite(str(salida / nombre), imagen):
        raise SystemExit(f"No se puede guardar {nombre}.")

print(f"\nFotos utilizadas: {len(puntos_fotos)} de {len(nombres)}. Error RMS: {error:.3f} píxeles.")
print(f"Resultados en: {salida}")
print("Calibración experimental de esta cámara. Revisa el resultado antes de usarlo.")
