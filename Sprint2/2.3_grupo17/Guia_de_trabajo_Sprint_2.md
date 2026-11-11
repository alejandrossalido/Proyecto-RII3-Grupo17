# Guía de trabajo — Sprint 2

**Proyecto:** PRII3 — Robots Inteligentes  
**Grupo:** 17  
**Documento de requisitos:** `PRII3+-+Sprint+2.pdf`

Esta guía distingue entre los requisitos que aparecen en el documento del sprint y las recomendaciones de organización del equipo.

## Objetivo del sprint

Al finalizar el sprint, el equipo debe poder controlar el movimiento del robot y evitar obstáculos mediante el Lidar, tanto en simulación como con el robot real. También debe calibrar la cámara del robot e identificar códigos ArUco.

## Requisitos del documento

### PBI 2.0 — Paquetes ROS

Crear estos paquetes, sustituyendo `XX` por el número del grupo:

- `g17_prii3_move_jetbot`: programas relacionados con el Jetbot real.
- `g17_prii3_move_turtlebot`: programas relacionados con la simulación del Turtlebot.

### PBI 2.1 — Movimiento autónomo (0,15 puntos)

Implementar un nodo `draw_number` para el robot real y otro para simulación. Ambos deben publicar los mensajes necesarios para dibujar autónomamente el número del grupo. Añadir un servicio ROS que permita detener, reanudar o reiniciar el dibujo.

### PBI 2.2 — Evitación de obstáculos (0,15 puntos)

Implementar para el robot real y para simulación los dos comportamientos siguientes:

1. `collision_avoidance`: detectar un obstáculo delante, detener el robot y reanudar la marcha cuando desaparezca.
2. `obstacle_avoidance`: esquivar el obstáculo sin detener la marcha.

Los dos nodos deben dibujar el número del grupo e incorporar su algoritmo de evitación correspondiente.

### PBI 2.3 — Cámara y ArUco (0,10 puntos)

- Demostrar la rectificación de la imagen del tablero Eurobot 2026 mediante la calibración de la cámara del robot.
- Identificar códigos ArUco con la cámara del robot.

### PBI 2.4 — Repositorio Git (0,10 puntos)

Consolidar la versión final del workspace ROS en la rama principal. El profesor debe poder descargar el código y ejecutarlo en su PC siguiendo las instrucciones del README.

### PBI 2.5 — Sprint Review

Preparar una presentación que incluya una explicación técnica y la demostración funcional de los apartados. Los profesores preguntarán a cada integrante. Según el documento:

- Primera respuesta correcta: nota individual igual a la nota del grupo.
- Primera incorrecta y segunda correcta: nota individual igual a la nota del grupo multiplicada por 0,7.
- Dos respuestas incorrectas: nota individual igual a la nota del grupo multiplicada por 0,4.

### Fechas indicadas

- **22 de octubre de 2026:** review del apartado 2.3 (cámara y ArUco).
- **29 de octubre de 2026:** review del resto, susceptible de modificación.

## Acceso al robot y workspace en VS Code

Estas instrucciones se han añadido a partir del material de la asignatura sobre trabajo remoto con el Jetbot.

### Conexión SSH desde VS Code

1. En VS Code, abre Quick Open con `Ctrl+P`, escribe y ejecuta:

   ```text
   ext install ms-vscode-remote.remote-ssh
   ```

2. Abre la paleta de comandos con `F1` o `Ctrl+Shift+P` y selecciona **Remote-SSH: Connect to Host...**.
3. Introduce el destino SSH con el formato `jetbot@ip_address`, sustituyendo `ip_address` por la dirección IP del robot.
4. Cuando VS Code solicite la contraseña, las instrucciones de la asignatura indican `jetbot`.
5. Una vez conectado, selecciona **Open Folder** y abre la carpeta del workspace ROS 2 que vayáis a usar en el robot.

La página de la asignatura con las instrucciones es [Trabajar con el robot (SSH remoto) y Workspace en VS Code](https://poliformat.upv.es/portal/site/GRA_14618_2026/tool/a429d21c-0238-4362-9e46-638868f86e3d/ShowPage?returnView=\&studentItemId=0\&backPath=\&bltiAppStores=false\&errorMessage=\&clearAttr=\&messageId=\&source=\&title=\&newTopLevel=false\&sendingPage=8005912\&postedComment=false\&addBefore=\&itemId=9943060\&path=push\&reviewAssessment=false\&topicId=\&addTool=-1\&recheck=\&id=\&forumId=#collapsibleContent).

## Simulación con TurtleBot3

### Instalación indicada para ROS 2 Humble

El material de la asignatura indica instalar los paquetes TurtleBot3 con:

```bash
sudo apt install ros-humble-turtlebot3*
```

En el entorno de este equipo, VS Code ya tiene instalada la extensión **Remote - SSH** y los paquetes TurtleBot3 se instalaron en la Distrobox `ros-humble`. Para trabajar en ese entorno:

```bash
distrobox enter --clean-path --name ros-humble
source /opt/ros/humble/setup.bash
ros2 pkg list | grep turtlebot3
```

### Plataforma y sensores

TurtleBot3 es una plataforma móvil de tracción diferencial: dos ruedas motrices independientes permiten avanzar, retroceder y girar. Su simulación permite desarrollar y validar algoritmos de navegación y evitación antes de probarlos en un robot real.

Sensores descritos en el material de la asignatura:

- **LiDAR:** publica medidas de distancia, normalmente mediante `sensor_msgs/LaserScan`. Es el sensor principal para detectar obstáculos cercanos; el nodo del sprint puede usar las lecturas frontales para decidir cuándo detenerse o iniciar una maniobra de esquiva.
- **Cámara RGB:** disponible en algunas configuraciones, como Waffle; permite procesar imágenes con visión artificial.
- **Encoders:** miden el giro y desplazamiento de las ruedas, y permiten estimar la odometría.
- **IMU:** mide aceleración y velocidad angular, y ayuda a estimar el estado del robot junto con otros sensores.

Para los PBIs 2.1 y 2.2, centrad primero las pruebas de movimiento en la interfaz de velocidad y las lecturas del LiDAR; la cámara del TurtleBot3 es información de contexto y no sustituye el requisito de calibrar e identificar ArUco con la cámara del robot indicado para el PBI 2.3.

## Lanzar y probar mundos TurtleBot3

El material de la asignatura propone tres mundos para iniciar Gazebo. Ejecuta los comandos dentro de la Distrobox `ros-humble`, con ROS Humble cargado. Antes de cambiar de mundo, detén por completo el actual con `Ctrl+C`.

Abre la Distrobox desde una terminal del sistema con `distrobox enter --clean-path --name ros-humble`. Una vez dentro, no vuelvas a ejecutar `distrobox enter`; ejecuta directamente los comandos ROS. `--clean-path` evita que se use por accidente el Python de Omarchy en lugar del Python de Ubuntu/ROS.

### 1. Mundo vacío

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Mundo TurtleBot3

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 3. Casa TurtleBot3

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

La primera ejecución de TurtleBot3 House puede tardar unos minutos mientras descarga el mapa.

Si `spawn_entity.py` falla con `ModuleNotFoundError: No module named 'lxml'`, instala la dependencia dentro de la Distrobox desde una terminal del sistema con `distrobox enter --clean-path --name ros-humble -- sudo apt install -y python3-lxml` y vuelve a lanzar el mundo.

### Detener Gazebo

Primero intenta cerrar la simulación con `Ctrl+C` en la terminal donde se lanzó. Si Gazebo se queda bloqueado o no carga, el material indica:

```bash
pkill gzclient
pkill gzserver
```

### Teleoperar y observar `/cmd_vel`

Para familiarizarse con el movimiento, la asignatura recomienda empezar en el mundo vacío. Los comandos de velocidad se publican en `/cmd_vel` como mensajes `geometry_msgs/msg/Twist`: `linear.x` controla el avance y `angular.z` el giro.

Observar los comandos publicados:

```bash
ros2 topic echo /cmd_vel
```

Publicar avance a 0,1 m/s:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Para probar otras velocidades, cambia `linear.x` o `angular.z`. También se puede usar `rqt_publisher`:

```bash
ros2 run rqt_publisher rqt_publisher
```

O la teleoperación de teclado:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Actividad del sprint

El nodo `draw_number` debe estar dentro de `g17_prii3_move_turtlebot`, publicar la trayectoria del número 17 y ofrecer servicios para detener, reanudar y reiniciar el dibujo. La ejecución del programa debe estar disponible mediante un único archivo `launch`. El workspace debe poder clonarse desde el repositorio tanto en el PC como en el robot, siguiendo el procedimiento de la asignatura.

## Estado de partida observado

En `Sprint1/g17_PRII3_ws` hay un paquete `g17_prii3_turtlesim` para dibujar el número 17 en `turtlesim`, con servicios para detener, reanudar y reiniciar el dibujo. Puede servir como referencia para la lógica del movimiento y los servicios. En la inspección inicial no se encontraron todavía los paquetes ni los nodos específicos del Sprint 2.

## Plan recomendado

El siguiente orden es una recomendación de organización, no un requisito adicional del PDF.

### 1. Preparación inmediata

- Confirmar acceso al Jetbot, a su cámara y al Lidar.
- Crear y compilar los paquetes `g17_prii3_move_jetbot` y `g17_prii3_move_turtlebot`.
- Acordar quién desarrolla cada parte y quién integra los cambios en el workspace común.
- Mantener un registro breve de cómo lanzar cada nodo y qué tópicos, servicios y sensores necesita.

### 2. Prioridad hasta el 22 de octubre: cámara y ArUco

- Calibrar la cámara y guardar los parámetros obtenidos.
- Rectificar una imagen del tablero Eurobot 2026.
- Detectar códigos ArUco.
- Guardar capturas o vídeos que demuestren ambos resultados.

Conviene empezar esta parte pronto porque tiene una fecha de review anterior a la del resto.

### 3. Movimiento autónomo

- Implementar primero el dibujo del 17 en simulación.
- Añadir y comprobar los servicios para parar, reanudar y reiniciar.
- Adaptar el control al Jetbot y validar el movimiento con el robot real.

### 4. Evitación de obstáculos

- Implementar y demostrar primero la parada y reanudación ante un obstáculo.
- Implementar después la maniobra para esquivarlo sin detener la marcha.
- Probar y documentar cada comportamiento tanto en simulación como en el Jetbot.
- Mantener demostraciones separadas para que se vea claramente la diferencia entre ambos algoritmos.

### 5. Integración y preparación de la review

- Dejar en la rama principal el workspace que se va a evaluar.
- Redactar un README desde el punto de vista de alguien que descarga el repositorio en un PC limpio: dependencias, compilación, lanzamiento y pruebas.
- Preparar una presentación técnica y una demostración reproducible de cada apartado.
- Asegurarse de que cada integrante pueda explicar los nodos, los sensores utilizados y las decisiones principales del algoritmo.

## Seguimiento sugerido

Usar una tabla del equipo como esta. Los responsables y las fechas internas los decide el grupo.

| Funcionalidad | Entorno | Responsable | Estado | Evidencia / comando de ejecución |
|---|---|---|---|---|
| Dibujo del 17 y servicios | Simulación |  | Pendiente |  |
| Dibujo del 17 y servicios | Jetbot real |  | Pendiente |  |
| Detenerse ante obstáculo y reanudar | Simulación |  | Pendiente |  |
| Detenerse ante obstáculo y reanudar | Jetbot real |  | Pendiente |  |
| Esquivar sin detener la marcha | Simulación |  | Pendiente |  |
| Esquivar sin detener la marcha | Jetbot real |  | Pendiente |  |
| Calibración y rectificación de Eurobot 2026 | Cámara del robot |  | Pendiente |  |
| Identificación de ArUco | Cámara del robot |  | Pendiente |  |
| README y rama principal preparados | Repositorio |  | Pendiente |  |
| Presentación y demostración | Sprint Review |  | Pendiente |  |

Se recomienda considerar una tarea terminada cuando el comportamiento se haya probado y exista una forma clara de repetir la demostración.
