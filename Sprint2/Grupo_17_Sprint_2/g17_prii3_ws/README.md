# Grupo 17 · Sprint 2

Robot móvil que dibuja el 17 con ROS 2. Incluye simulación, JetBot real y cámara.

## Carpeta

- `src/g17_prii3_move_jetbot`: robot físico. `controller.py` gestiona ROS y los
  tres modos, `motion.py` contiene el movimiento y el rodeo, y `diagnostics.py`
  comprueba los sensores sin mover motores. `test/` conserva las pruebas.
- `src/g17_prii3_move_turtlebot`: simulación original en Gazebo.
- `camera/`: un único programa de cámara, calibración y fotos para recalibrar.
- `entorno_robot.sh`: carga el entorno del JetBot.

Los archivos `package.xml`, `setup.py`, `setup.cfg`, `resource/` y `launch/`
son necesarios para registrar y ejecutar los paquetes ROS.

## Compilar

Desde esta carpeta, en Ubuntu con ROS y `colcon` instalados:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`build/`, `install/` y `log/` se regeneran al compilar; no forman parte del código.

En el JetBot, cargar primero el workspace del fabricante si es necesario.
Se necesita `jetbot_pro_ros2`. `source ./entorno_robot.sh` selecciona el ROS
instalado (Foxy o Humble), carga esta entrega y establece dominio 17 con ROS
solo local. Las pruebas de esta revisión se realizan en Humble.

## JetBot real

Conectar normalmente por SSH: `ssh jetbot@172.16.190.172` e introducir la
contraseña. El workspace del grupo en el robot es `~/g17_prii3_ws`; el driver
del fabricante está en `~/ros2_ws` y `entorno_robot.sh` lo carga si existe.
No se han configurado claves exclusivas ni restricciones de acceso.

Ejecutar **un solo modo** en el JetBot:

```bash
cd ~/g17_prii3_ws
source ./entorno_robot.sh
ros2 launch g17_prii3_move_jetbot jetbot.launch.py mode:=draw_number
```

Cambiar `mode` según el comportamiento:

| Modo | Función |
|---|---|
| `draw_number` | Dibuja el 17 con odometría. No usa LiDAR. |
| `collision_avoidance` | Se para ante un objeto y continúa el mismo dibujo cuando se retira. |
| `obstacle_avoidance` | Rodea el objeto y termina el 17 ya iniciado. Si no cabe, espera. |

Los tres empiezan pausados. En otra terminal **SSH del mismo robot**:

```bash
source ./entorno_robot.sh
ros2 service call /continue_drawing std_srvs/srv/Trigger '{}'
ros2 service call /stop_drawing std_srvs/srv/Trigger '{}'
ros2 service call /restart_drawing std_srvs/srv/Trigger '{}'
```

`continue` reanuda, `stop` pausa y `restart` borra el dibujo solo cerca de su
inicio original y lo deja pausado. Los obstáculos nunca reinician el 17.
`waypoint` conserva el progreso, `detour` guarda el rodeo y `detour_goal` indica
dónde reincorporarse. Si un objeto ocupa un vértice, intenta otro posterior.

El lanzamiento inicia también el chasis. Si ya está arrancado con el mismo
entorno, añadir `start_chassis:=false`. No duplicar el driver ni ejecutar
teleoperación a la vez.

### Sensores y espacio

Antes de mover, con el controlador pausado:

```bash
ros2 topic list -t
ros2 run g17_prii3_move_jetbot diagnose_robot --duration 8
```

Temas: `/wheel/odometry` (posición), `/scan` (LiDAR), `/cmd_vel` (velocidad) y
`/g17/drawing` (traza en RViz). Si el robot publica `/odom`, añadir
`odom_topic:=/odom` al lanzamiento y `--odom-topic /odom` al diagnóstico.
El rodeo necesita LiDAR de 360°. Un único publicador por sensor y controlador.

El dibujo mide 44 × 30,46 cm y el recinto fijo es **1 × 1 m**. Colocar el robot
**antes de lanzar**: centro a 28 cm del borde trasero y 28 cm del izquierdo,
mirando paralelo al cuadrado. Avanza hacia delante y dibuja hacia su derecha.
Control a 20 Hz, máximo 0,12 m/s y 0,65 rad/s, mínimo inicial 0,06 m/s.

Medir el radio envolvente real desde `base_link` (`robot_radius:=0.18` por
defecto). Ajustar `linear_scale`, `scan_yaw_offset`, `scan_x` y `scan_y` solo con
mediciones. La odometría y el frenado deben coincidir con el movimiento real:
el límite calculado no garantiza el espacio físico si esos datos son erróneos.
Los fallos de sensores o de movimiento quedan enclavados hasta corregirlos.

## Simulación

Elegir un comando y cerrar el anterior antes del siguiente:

```bash
ros2 launch g17_prii3_move_turtlebot simulation.launch.py mode:=draw_number rviz:=true
ros2 launch g17_prii3_move_turtlebot simulation.launch.py mode:=collision_avoidance spawn_cube:=true auto_remove_cube:=true rviz:=true
ros2 launch g17_prii3_move_turtlebot simulation.launch.py mode:=obstacle_avoidance spawn_cube:=true rviz:=true
```

Requiere TurtleBot3 y Gazebo. En RViz, marco `odom` y marcador `/g17/drawing`.
La simulación conserva su código original y `restart` toma la posición actual.

## Cámara

Requiere OpenCV con `cv2.aruco` y NumPy. Los tres programas anteriores se unifican:

```bash
python3 camera/camara.py calibrar
python3 camera/camara.py rectificar
python3 camera/camara.py aruco
```

Los dos últimos admiten una foto: `python3 camera/camara.py aruco foto.jpg`.
Guardan las imágenes en `camera/resultados/`. La calibración utiliza las fotos
incluidas y solo vale para la cámara y resolución con que se tomaron. Los ejes
ArUco mantienen las unidades del marcador del código original.

## Pruebas y presentación

Tras compilar y cargar el workspace:

```bash
python3 -m pytest src/g17_prii3_move_jetbot/test -q -p no:cacheprovider
```

Las pruebas aíslan sus temas y usan dominio 231 con un chasis ficticio. No
sustituyen medir precisión, sensores y frenado en el JetBot de clase.

[Presentación de Google Slides](https://docs.google.com/presentation/d/1YpTFqDaq7DGIy06xF20cjilZvTns5JZM-yuS0SZpS-I)
