# Guion de exposición · Grupo 17

Duración orientativa: 3 a 4 minutos, más la demostración. Abrir `Presentacion_Grupo_17.pptx` y tener preparado el entorno de Python del README.

1. **Portada.** «En el apartado 2.3 hemos calibrado la cámara, aplicado la corrección de distorsión y detectado marcadores ArUco en las fotos del robot».
2. **Trabajo realizado.** Explicar que la calibración proporciona los parámetros para corregir imágenes y estimar la orientación de los marcadores.
3. **Calibración.** «Partimos de 51 fotos. Encontramos el patrón completo en 22 y utilizamos 14 tras filtrar errores. El ajedrez tiene 13 por 9 esquinas interiores y el error RMS final es de 1,97 píxeles. Guardamos los parámetros en un YAML».
4. **Corrección.** «Aplicamos `undistort` con la matriz y la distorsión guardadas. En este ejemplo la desviación de rectitud baja de 0,759 a 0,503 píxeles. La mejora es pequeña y no aparece en todas las fotos. El papel ondulado limita el ajuste».
5. **ArUco.** «Detectamos los identificadores y estimamos la pose para dibujar los ejes. En esta foto aparecen 20, 21, 23 y 47. Podemos recorrer las fotografías». Ejecutar `python demo.py`, cerrar la comparación con una tecla y avanzar unas fotos con N. Salir con Q.
6. **Conclusiones.** «Tenemos el proceso completo y reproducible con las fotos disponibles. Para mejorar la precisión repetiríamos la captura con un patrón rígido y plano y comprobaríamos el resultado con fotografías nuevas».

## Preguntas que todos deberíamos saber responder

- **¿Qué guarda el YAML?** La matriz K, los coeficientes D, la resolución y datos del ajuste.
- **¿Qué diferencia hay entre calibrar y rectificar?** Calibrar estima los parámetros de la cámara. Rectificar aplica la corrección de distorsión a una imagen.
- **¿Qué significa RMS?** La raíz de la media de los errores cuadrados. En calibración mide la distancia entre las esquinas detectadas y sus proyecciones, en píxeles.
- **¿Por qué 14 fotos si hay 51?** En 29 no detectamos el patrón completo. De las 22 restantes descartamos 8 por error de reproyección superior a 3 px durante el filtrado.
- **¿Por qué el cambio visual es pequeño?** Depende de la distorsión de la lente, la posición del patrón y la calidad de las fotos. La curvatura del papel también influye y no la corrige `undistort`.
- **¿Qué representa un ID ArUco?** Identifica el patrón dentro del diccionario usado. No es una distancia ni una posición.
- **¿Qué representan los ejes?** La orientación estimada del marcador respecto a la cámara: X rojo, Y verde y Z azul.
- **¿Medimos distancias en metros?** No. Hemos usado un lado de marcador igual a una unidad. Habría que introducir la medida real para obtener unidades físicas.
- **¿Es vídeo en directo?** Esta demostración procesa las fotos guardadas del robot.
- **¿La validación es independiente?** No. Hemos comprobado el comportamiento en el conjunto disponible, que también interviene en la calibración.

## Antes de empezar

Activar el entorno del README, ejecutar las pruebas y abrir la presentación. Durante la review basta con `python demo.py`; no recalibrar en directo. Si no hay interfaz gráfica, mostrar las capturas de `resultados/`.
