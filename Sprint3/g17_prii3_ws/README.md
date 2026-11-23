# Sprint 3 · Grupo 17

Simulación en Gazebo con ROS 2 Humble y TurtleBot3. El robot navega por F1L3,
llega a la puerta, detecta un ArUco y continúa al destino asociado.

## Preparar y compilar

Se necesita Ubuntu 22.04 con ROS 2 Humble. En nuestro ordenador usamos Ubuntu
en Distrobox: `distrobox enter --clean-path --name ros-humble`.
Si ya estás dentro de Ubuntu, sigue directamente estos pasos.

Instalar las dependencias:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-opencv python3-numpy python3-yaml ros-humble-cv-bridge ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-turtlebot3-gazebo ros-humble-teleop-twist-keyboard
```

Descargar el repositorio y compilar solo este workspace:

```bash
git clone https://github.com/alejandrossalido/Proyecto-RII3-Grupo17.git
cd Proyecto-RII3-Grupo17/Sprint3/g17_prii3_ws
source entorno.sh
colcon build --symlink-install
source entorno.sh
```

## Ejecutar

Abrir la simulación:

```bash
ros2 launch g17_prii3_nav_turtlebot sprint3.launch.py mode:=autonomous marker_id:=10
```

Esperar a que RViz indique Navigation y Localization en verde. En otra terminal,
entrar también en Ubuntu, situarse en esta carpeta y cargar `source entorno.sh`.
Iniciar la misión y ver su estado:

```bash
ros2 service call /g17/mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /g17/mission/status
```

El recorrido es: **inicio → puerta → lectura del ArUco → destino → done**.
Se pueden probar los IDs 10, 20 y 30. Cerrar el lanzamiento con Ctrl+C antes de
iniciar otra simulación. Para cancelar una misión:

```bash
ros2 service call /g17/mission/cancel std_srvs/srv/Trigger '{}'
```

## Otros modos de simulación

Sustituir `mode:=autonomous marker_id:=10` por:

- `mode:=guided`: seleccionar un destino con Nav2 Goal en RViz.
- `mode:=predefined`: ir al destino fijo; iniciar con el mismo servicio.
- `mode:=mapping`: crear un mapa con SLAM y recorrer el mundo con teleoperación.

Para mover el robot en el modo mapping, desde otra terminal preparada:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.4
```

Cerrar la teleoperación antes de enviar objetivos de navegación. Para guardar
el mapa creado con SLAM:

```bash
ros2 run nav2_map_server map_saver_cli --free 0.196 -f "$PWD/mapa_slam" --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

## Qué hace cada archivo

Los módulos Python están en `src/g17_prii3_nav_common/g17_prii3_nav_common/`:

| Archivo | Función |
|---|---|
| `mission.py` | Envía objetivos a Nav2 y cambia de paso mediante callbacks. |
| `aruco.py` | Recibe las imágenes y publica los IDs y sus centros en píxeles. |
| `vision.py` | Detecta los marcadores con OpenCV. |
| `route.py` | Carga las poses y confirma el ID en tres imágenes recientes. |
| `bringup.py` | Arranca los nodos y configura Nav2 según el modo elegido. |
| `simulation.py` | Crea el mundo de Gazebo, el cartel y el robot. |
| `map_tools.py` | Lee el mapa y obtiene las paredes para construir el mundo. |

Nav2 calcula las rutas y controla el movimiento. Nuestro nodo de misión le
indica a dónde ir; no implementa un planificador propio.

El paquete `g17_prii3_nav_turtlebot` contiene el launch `sprint3.launch.py` y
`config/poses.yaml`, con el inicio, la puerta y los tres destinos.
En el paquete común, `config/navigation.yaml` ajusta Nav2 y `config/sprint3.rviz`
configura la vista. El mapa es `maps/f1l3_nav.yaml` junto con `f1l3_sim.pgm`.
El YAML mantiene las zonas grises como desconocidas. Gazebo reconstruye las
paredes negras con 1 m de altura; no es un mundo 3D oficial del profesor.

`package.xml`, `setup.py`, `setup.cfg`, `resource/` y `__init__.py` son los
archivos que ROS 2 y Python necesitan para reconocer e instalar los paquetes.
`entorno.sh` carga ROS y este workspace. `.gitignore` excluye archivos generados.
Las carpetas `build`, `install` y `log` aparecen al compilar y están ocultas en
el archivo de VS Code `Simulacion.code-workspace`.
