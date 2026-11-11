# Comprobación de la rectificación

Se ha aplicado a otras fotos la misma operación de `rectificar.py`: `cv2.undistort(imagen, K, D)`, con los parámetros existentes en `Entrega_2.3/calibracion_jetbot.yml`. No se han modificado los scripts, la calibración ni los resultados originales de la entrega.

## Resultados

La medida es la distancia RMS perpendicular de las esquinas del ajedrez a la mejor recta de cada fila y columna, en píxeles de la imagen de 1920 × 1080. Se detectan las esquinas en la foto original y se transforman con `undistortPoints(..., P=K)` para evaluar las mismas correspondencias tras la corrección. Menor valor significa líneas más rectas en ese patrón, no necesariamente una calibración global mejor. No es el error RMS de calibración, y las variaciones de escala de la transformación también influyen en una medida expresada en píxeles.

| Foto (hora del nombre) | Original | Corregida | Lectura |
|---|---:|---:|---|
| 15:09:04, ejemplo original | 0,376 px | 0,389 px | Diferencia mínima |
| 15:09:26 | 0,759 px | 0,503 px | Mejora, aunque pequeña en términos absolutos |
| 15:09:35 | 2,540 px | 3,111 px | Empeora; la hoja aparece ondulada |
| 15:09:45 | 0,462 px | 0,372 px | Mejora leve |

Se encontró el patrón completo en 22 de las 51 fotografías con el procedimiento del script. De esas 22, la rectitud mejora en 13. La mediana pasa de 1,392 a 1,323 px. Las fotografías proceden del mismo conjunto de calibración; no es una validación independiente. Las mediciones y recortes no permiten aislar por sí solos la distorsión óptica de la curvatura física del papel.

La hoja se ve doblada en varias fotos. El algoritmo de calibración supone un patrón plano, por lo que esta es una fuente plausible de sesgo. La elección de un patrón cerca del centro en el ejemplo original también hace menos demostrativa la comparación. La aplicación de `undistort` es coherente con la API y con el YAML; las pruebas no identifican un fallo en `rectificar.py`.

## Problema adicional del código de calibración en este entorno

Con OpenCV 5.0.0, `findChessboardCornersSB` devuelve aquí esquinas de forma `(117, 2)` y `projectPoints` devuelve `(117, 1, 2)`. La expresión de `calibrar.py`, `esquinas - calculadas`, produce por broadcasting un array `(117, 117, 2)` y compara todos los pares, en lugar de cada esquina con su proyección. Eso altera los errores usados para filtrar las fotografías.

En una comprobación con la primera foto y la calibración guardada, tras estimar la pose mediante `solvePnP`, la expresión actual devuelve 191,858 px y la resta punto a punto devuelve 2,163 px. Esta comparación sirve para demostrar el problema de formas, no para reproducir el ajuste que generó el YAML.

Una forma compatible de calcular ese error es:

```python
diferencias = esquinas.reshape(-1, 2) - calculadas.reshape(-1, 2)
error = np.sqrt(np.mean(np.sum(diferencias ** 2, axis=1)))
```

El fallo depende de las formas devueltas por la versión instalada. No demuestra que el YAML ya entregado se generase con ese fallo: si ambas salidas tenían forma `(117, 1, 2)`, la expresión original era válida. Corregir el cálculo del error tampoco cambia por sí solo las imágenes ya rectificadas con el YAML existente.

## Archivos

- `caso_*.jpg`: comparaciones de imagen completa y recortes del ajedrez, siempre original a la izquierda y corregida a la derecha. Los recortes usan una ventana común y conservan la misma escala entre lados. Las imágenes se reducen o amplían para visualizarlas; las medidas se calculan a resolución original.
- `mediciones.csv`: medidas de las 22 fotografías y ajuste adicional a una cuadrícula plana mediante homografía.
- `resumen.json`: resultados agregados y auditoría de las formas de los arrays.
- `comprobar.py`: programa reproducible. Ejecutar con Python, NumPy y OpenCV desde cualquier carpeta.

Referencia del método: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
