# Ficha técnica — Proyecto RII 3: Robots Inteligentes · Sprint 2

## Objetivo del Sprint
Al finalizar, cada equipo debe:
- Controlar el movimiento del robot en simulación y robot real.
- Implementar evitación de obstáculos mediante sensor LiDAR.
- Calibrar la cámara e identificar códigos ArUco.
- Validar todo en Turtlebot simulado y Jetbot real.

## Paquetes ROS requeridos
| Plataforma | Paquete |
|---|---|
| Jetbot real | `gXX_prii3_move_jetbot` |
| Turtlebot simulado | `gXX_prii3_move_turtlebot` |

`XX` = número del grupo.

## PBI 2.1 — Movimiento autónomo
Nodo: `draw_number`
- Un nodo para Jetbot y otro para Turtlebot.
- Publica los mensajes de velocidad necesarios para dibujar autónomamente el número del grupo.
- Debe incluir un servicio ROS con tres operaciones:
  - Detener el dibujo.
  - Reanudar el dibujo.
  - Reiniciar el dibujo.

## PBI 2.2 — Evitación de obstáculos
### `collision_avoidance`
- Implementa detección frontal mediante LiDAR.
- Si detecta un obstáculo delante: detiene el robot.
- Cuando el obstáculo desaparece: reanuda la marcha.
- Comportamiento reactivo con parada de seguridad.

### `obstacle_avoidance`
- Detecta obstáculos mediante LiDAR.
- Permite esquivarlos sin detener completamente el robot.
- Debe mantener el movimiento modificando la trayectoria.

## PBI 2.3 — Cámara y ArUco
- Calibrar la cámara del robot.
- Rectificar la imagen del tablero Eurobot 2026.
- Detectar e identificar códigos ArUco.

**Diferencia clave:** calibración estima parámetros de cámara; rectificación corrige la distorsión; identificación determina el marcador ArUco detectado.

## PBI 2.4 — Git
- Workspace ROS consolidado en la rama principal.
- `README` obligatorio con instrucciones reproducibles de instalación y ejecución.
- El profesor debe poder descargar y ejecutar el proyecto en su PC.

## Evaluación y fechas
- PBI 2.1: 0,15 puntos.
- PBI 2.2: 0,15 puntos.
- PBI 2.3: 0,10 puntos.
- PBI 2.4: 0,10 puntos.
- Total: 0,5 puntos.
- Sprint Review: apartado 2.3 el 22 de octubre; resto el 29 de octubre, sujeto a modificación.
- Respuestas: correcta → 100%; primera incorrecta y segunda correcta → 70%; dos incorrectas → 40%.
