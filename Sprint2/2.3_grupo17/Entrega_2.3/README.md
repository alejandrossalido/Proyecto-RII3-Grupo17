# Grupo 17 · Sprint 2 · Apartado 2.3

Calibración de la cámara, corrección de distorsión e identificación de códigos ArUco en las fotografías del robot del tablero Eurobot 2026.

## Contenido de la entrega

- `Presentacion_Grupo_17.pptx`: presentación final de 6 diapositivas.
- `calibrar.py`: calcula la matriz de cámara y la distorsión, filtra fotografías y guarda el YAML y un informe con los errores por foto.
- `rectificar.py`: compara una fotografía original con su corrección mediante `cv2.undistort`.
- `detectar_aruco.py`: identifica ArUco y muestra sus ejes al recorrer las fotos.
- `vision.py`: funciones comunes y comprobaciones de entradas.
- `comprobar.py`: mide la rectitud de filas y columnas del ajedrez antes y después de la corrección.
- `demo.py`: ejecuta las demostraciones.
- `ejecutar_demo.sh`: lanza la demostración en Linux con el entorno `.venv`.
- `calibracion_jetbot.yml`: calibración reproducida con el código corregido.
- `Fotos_Sprint2_2.3/`: las 51 fotografías originales.
- `resultados/`: capturas, mediciones e informes reproducibles.
- `pruebas/`: pruebas de regresión.
- `GUIA_EXPOSICION.md`: guion breve y preguntas para preparar la defensa.

## Instalación

Usar **Python 3.10 a 3.13**. El entorno comprobado es **Python 3.12.14, OpenCV 4.13.0 y NumPy 2.2.6**, en Linux x86_64. Las dependencias fijan versiones para reproducir los resultados. Hace falta conexión a internet solo para instalarlas.

Desde esta carpeta, en Linux o macOS:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Si el ejecutable tiene otro nombre, sustituir `python3.12` por el de una versión compatible. En Windows, crear el entorno con `py -3.12 -m venv .venv` y activarlo con `.venv\Scripts\Activate.ps1` en PowerShell.

No instalar a la vez `opencv-python`, `opencv-contrib-python` ni sus variantes `headless` dentro del mismo entorno, porque todos proporcionan el módulo `cv2`. El entorno nuevo anterior evita esa mezcla. La versión incluida en `requirements.txt` permite las ventanas de demostración.

**No actualizar a OpenCV 5 para esta entrega.** Se comprobaron diferencias tanto en el ajuste de cámara como en la detección. Las llamadas ArUco del código usan la API moderna, pero los resultados de referencia y las pruebas corresponden a la versión fijada. Python 3.14 queda fuera del entorno fijado por la versión de NumPy.

## Demostración para la review

La calibración ya está incluida. No es necesario recalcularla durante la presentación.

```bash
python demo.py
```

En Linux también se puede ejecutar `bash ejecutar_demo.sh`, que utiliza directamente el entorno `.venv` creado en esta carpeta. El ZIP no incluye entornos virtuales: después de extraerlo hay que seguir la instalación anterior.

1. Aparece la comparación original/rectificada. Pulsar una tecla con la ventana enfocada para continuar.
2. Aparece la detección ArUco. **Espacio o N** avanza, **P** retrocede y **Q o Esc** termina. La primera foto contiene los IDs **20, 21, 23 y 47**.

Para ejecutar cada parte por separado:

```bash
python rectificar.py
python detectar_aruco.py
```

Las demostraciones utilizan **fotografías guardadas**, no una conexión de vídeo en directo. Las capturas incluidas permiten explicar los resultados si no hay interfaz gráfica disponible. No se necesita ROS para ejecutar estos scripts.

## Reproducir todos los resultados

Desde esta carpeta y con el entorno activado:

```bash
python calibrar.py
python demo.py --sin-ventana
python -m unittest discover -s pruebas -v
```

La calibración procesa las 51 fotos y puede tardar. `demo.py --sin-ventana` genera las capturas y los informes de detección y rectitud sin abrir ventanas. Los valores pueden variar ligeramente entre plataformas. Los scripts también funcionan desde otra carpeta si se indica su ruta, porque las rutas predeterminadas se calculan respecto a cada script.

Se pueden indicar otras entradas y salidas con `--fotos`, `--calibracion`, `--foto` o `--salida`, según el programa. Consultar `python nombre_del_script.py --help`. Las imágenes deben tener la resolución de la calibración, **1920 × 1080**. No utilizar este YAML con otra cámara, recorte, resolución o configuración de enfoque sin una comprobación adicional.

## Resultados de referencia

| Resultado | Valor |
| --- | --- |
| Fotografías disponibles | 51 |
| Fotografías con patrón completo | 22 |
| Fotografías del ajuste final | 14 |
| Patrón de esquinas interiores | 13 × 9, 117 puntos |
| Resolución | 1920 × 1080 |
| RMS de reproyección del ajuste final | 1,969438 px |
| Umbral de filtrado por fotografía | 3 px |
| Familia de ArUco | DICT_4X4_50 |
| IDs en la foto de demostración | 20, 21, 23 y 47 |
| Fotografías con alguna detección ArUco | 51 de 51 |
| Fotografías con mejora de rectitud | 13 de las 22 con patrón completo |
| Mediana de rectitud antes/después | 1,392 / 1,323 px |

`resultados/calibracion.json` identifica todas las fotos usadas, descartadas y sin patrón. `resultados/deteccion_aruco.json` registra los IDs, las poses y el error de reproyección de cada detección. `resultados/mediciones_rectitud.csv` contiene las medidas por foto y `comprobacion_rectitud.json` las resume.

La comparación principal usa la foto `WIN_20260924_15_09_26_Pro.jpg`, un ejemplo con mejora pequeña: **0,759 a 0,503 px**. Se incluyen también los recortes del ejemplo original (cambio mínimo, **0,376 a 0,389 px**) y de un caso que empeora (**2,540 a 3,111 px**). Es una selección ilustrativa, no una demostración de mejora en todas las fotografías.

## Explicación y limitaciones

**Calibración:** relacionamos las 117 esquinas del ajedrez con sus posiciones en la imagen para obtener la matriz de cámara K y los coeficientes de distorsión D. Calculamos cada error con correspondencias punto a punto, normalizando la forma de los arrays para evitar comparaciones incorrectas. Tras cada filtrado volvemos a ajustar con las fotografías que quedan.

**Rectificación:** aplicamos `cv2.undistort(imagen, K, D)`. La operación corrige la distorsión estimada de la lente. No elimina la perspectiva ni aplana las ondulaciones físicas del papel.

**ArUco:** detectamos el ID y sus cuatro esquinas con `ArucoDetector`. Estimamos la pose mediante `solvePnP(..., SOLVEPNP_IPPE_SQUARE)` y dibujamos X en rojo, Y en verde y Z en azul. El lado del marcador vale una unidad: las traslaciones representan lados de marcador, **no metros**. Para distancias físicas habría que medir el lado real y usar esa unidad.

**Calidad de la calibración:** el papel aparece ondulado en varias fotos y el modelo supone un tablero plano. La rectificación mejora de forma limitada y empeora en algunas imágenes. El RMS de 1,97 px describe el ajuste sobre las imágenes usadas; no garantiza precisión global. La comprobación de rectitud usa fotos del mismo conjunto, por lo que **no es una validación independiente**. Mide distancia perpendicular de las esquinas a la mejor recta por fila y columna, en píxeles; también influye la escala de la transformación. No es el RMS de reproyección.

Para mejorar la precisión se necesitan nuevas fotos de un patrón **rígido y plano**, con distintas posiciones e inclinaciones, y reservar imágenes nuevas para validar. Esa nueva captura requiere acceso físico al robot y queda fuera de los resultados aquí demostrados.

## Referencias técnicas

- [Calibración de cámara en OpenCV](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)
- [Detección ArUco en OpenCV](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html)
- [Estimación de pose y orden de esquinas de IPPE_SQUARE](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)
