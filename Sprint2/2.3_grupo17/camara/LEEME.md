# Calibrar la cámara con las fotos

Abre una terminal en `Sprint2` y ejecuta:

```bash
python camara/calibrar.py
```

En este ordenador ya están instalados Python, NumPy y OpenCV. El programa
encuentra el ZIP automáticamente; no necesitas descomprimirlo ni pasar opciones.
También puedes ejecutarlo desde otra carpeta indicando la ruta del programa.

## Qué hace el código

Los seis bloques numerados de `calibrar.py` hacen lo siguiente:

1. Seleccionar el ZIP, la carpeta de salida y el patrón de 13 × 9 esquinas interiores.
2. Crear las coordenadas de las 117 esquinas del ajedrez plano.
3. Buscar esas esquinas en las fotos y utilizar las que se detectan completas.
4. Calcular los parámetros con `cv2.calibrateCamera`.
5. Guardar la matriz de cámara y la distorsión en `calibracion.yml`.
6. Corregir una foto y guardar la comparación original/corregida.

## Qué abrir después

En `camara/resultados_camara_normal` se generan:

- `calibracion.yml`: parámetros, resolución, número de fotos y error RMS.
- `comparacion.jpg`: la misma foto original a la izquierda y corregida a la derecha.
- `original.jpg` y `corregida.jpg`: las dos imágenes por separado.

Los archivos `informe.json` y `esquinas.jpg`, si están presentes, proceden de la
versión anterior. Esta versión sencilla no los genera ni los actualiza.

## Qué significa el resultado

El error RMS está en píxeles: indica el ajuste a las esquinas observadas, pero no
certifica la calidad. Hay que revisar la comparación y probar con otra captura
de la misma cámara que no se haya utilizado para calibrar.

El papel del ajedrez de estas fotos aparece curvado, lo que puede perjudicar el
cálculo. Para nuevas fotos, fíjalo a una superficie plana y rígida y varía su
posición e inclinación manteniendo cámara, resolución, enfoque y zoom.

Los parámetros pertenecen a la cámara normal que tomó estas fotos. Para una
cámara distinta en el robot hay que repetir la captura y la calibración; cambia
`archivo_zip` y `salida` al principio del programa y la etiqueta `camera_name`.

No hace falta conocer los milímetros de una casilla para calcular la matriz y
la distorsión: aquí se utiliza una casilla como unidad. Este programa no calcula
distancias en metros ni detecta ArUco.

Referencia: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
