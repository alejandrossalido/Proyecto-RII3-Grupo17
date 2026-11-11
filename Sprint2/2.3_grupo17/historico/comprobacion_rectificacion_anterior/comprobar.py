"""Comprueba la calibracion existente sin modificar los entregables originales."""
from pathlib import Path
import csv
import json
import cv2
import numpy as np

BASE = Path(__file__).resolve().parent
ENTREGA = BASE.parent / "Entrega_2.3"
cv2.setNumThreads(4)
datos = cv2.FileStorage(str(ENTREGA / "calibracion_jetbot.yml"), cv2.FILE_STORAGE_READ)
K = datos.getNode("camera_matrix").mat()
D = datos.getNode("distortion_coefficients").mat()
size = (int(datos.getNode("image_width").real()), int(datos.getNode("image_height").real()))
datos.release()
grid = np.mgrid[0:13, 0:9].T.reshape(-1, 2).astype(np.float32)
obj = np.column_stack((grid, np.zeros(len(grid)))).astype(np.float32)


def detectar(gris):
    ok, c = cv2.findChessboardCornersSB(cv2.resize(gris, None, fx=.5, fy=.5),
                                      (13, 9), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        c *= 2
        cv2.cornerSubPix(gris, c, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    else:
        ok, c = cv2.findChessboardCornersSB(gris, (13, 9), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return c if ok else None


def rectitud(puntos):
    a = puntos.reshape(9, 13, 2)
    errores = []
    for linea in list(a) + list(a.transpose(1, 0, 2)):
        vx, vy, x, y = cv2.fitLine(linea, cv2.DIST_L2, 0, .01, .01).ravel()
        errores.extend(((linea[:, 0] - x) * vy - (linea[:, 1] - y) * vx).tolist())
    return float(np.sqrt(np.mean(np.square(errores))))


def ajuste_plano(puntos):
    pts = puntos.reshape(-1, 2)
    H, _ = cv2.findHomography(grid, pts, 0)
    estimadas = cv2.perspectiveTransform(grid[:, None, :], H).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((estimadas - pts) ** 2, axis=1))))


def panel(imagen, titulo, ancho=960):
    foto = cv2.resize(imagen, (ancho, round(imagen.shape[0] * ancho / imagen.shape[1])))
    cabecera = np.full((54, ancho, 3), (30, 30, 30), np.uint8)
    cv2.putText(cabecera, titulo, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, .7,
                (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack((cabecera, foto))


seleccion = {"15_09_04": "caso_original", "15_09_26": "caso_mejora",
             "15_09_35": "caso_empeora", "15_09_45": "caso_mejora_leve"}
filas, detectadas = [], []
for ruta in sorted((ENTREGA / "Fotos_Sprint2_2.3").glob("*.jpg")):
    im = cv2.imread(str(ruta))
    if (im.shape[1], im.shape[0]) != size:
        raise ValueError(f"Resolucion incompatible: {ruta.name}")
    esquinas = detectar(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
    if esquinas is None:
        continue
    detectadas.append((ruta.name, esquinas))
    corregidas = cv2.undistortPoints(esquinas, K, D, P=K).reshape(-1, 2)
    antes, despues = rectitud(esquinas), rectitud(corregidas)
    fila = dict(foto=ruta.name, rectitud_original_px=antes, rectitud_corregida_px=despues,
                ajuste_plano_original_px=ajuste_plano(esquinas),
                ajuste_plano_corregido_px=ajuste_plano(corregidas))
    filas.append(fila)
    print(ruta.name, f"{antes:.3f} -> {despues:.3f} px", flush=True)
    for hora, nombre in seleccion.items():
        if hora not in ruta.name:
            continue
        und = cv2.undistort(im, K, D)
        cv2.imwrite(str(BASE / f"{nombre}.jpg"), np.hstack((
            panel(im, f"ORIGINAL - desviacion {antes:.3f} px"),
            panel(und, f"CORREGIDA - desviacion {despues:.3f} px"))))
        pts = np.vstack((esquinas.reshape(-1, 2), corregidas))
        x0, y0 = np.maximum(np.floor(pts.min(axis=0)).astype(int) - 35, 0)
        x1, y1 = np.minimum(np.ceil(pts.max(axis=0)).astype(int) + 35, size)
        cv2.imwrite(str(BASE / f"{nombre}_detalle.jpg"), np.hstack((
            panel(im[y0:y1, x0:x1], "ORIGINAL - detalle", 700),
            panel(und[y0:y1, x0:x1], "CORREGIDA - detalle", 700))))

with (BASE / "mediciones.csv").open("w", newline="") as archivo:
    escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
    escritor.writeheader()
    escritor.writerows(filas)

# Auditoria de las formas de arrays que usa calibrar.py en este OpenCV.
# Comparamos la expresion literal del entregable con la resta punto a punto.
nombre, c = detectadas[0]
ok, r, t = cv2.solvePnP(obj, c, K, D)
proyectadas, _ = cv2.projectPoints(obj, r, t, K, D)
literal = np.sqrt(np.mean(np.sum((c - proyectadas) ** 2, axis=2)))
correcto = np.sqrt(np.mean(np.sum((c.reshape(-1, 2) - proyectadas.reshape(-1, 2)) ** 2, axis=1)))
resumen = dict(opencv=cv2.__version__, fotos_con_patron=len(filas),
               mejoran=sum(f['rectitud_corregida_px'] < f['rectitud_original_px'] for f in filas),
               mediana_original_px=float(np.median([f['rectitud_original_px'] for f in filas])),
               mediana_corregida_px=float(np.median([f['rectitud_corregida_px'] for f in filas])),
               auditoria_error=dict(foto=nombre, forma_esquinas=list(c.shape),
                                   forma_proyectadas=list(proyectadas.shape),
                                   error_expresion_actual_px=float(literal),
                                   error_punto_a_punto_px=float(correcto)))
(BASE / "resumen.json").write_text(json.dumps(resumen, indent=2))
print(json.dumps(resumen, indent=2))
