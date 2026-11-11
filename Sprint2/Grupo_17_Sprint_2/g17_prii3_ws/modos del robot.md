# Modos del robot · Guía rápida

**Conectar las dos terminales → Elegir modo en la 1 → Empezar desde la 2**

| Modo | Qué hace |
|---|---|
| **1 · Dibujar** | Hace el 17 sin detectar obstáculos. |
| **2 · Parar** | Se detiene ante un objeto y sigue cuando lo retiras. |
| **3 · Esquivar** | Rodea el objeto y continúa el mismo 17. Si no cabe, espera. |

## 1️⃣ Conectar · En las DOS terminales

```bash
ssh jetbot@172.16.190.172
```

Introduce la contraseña. Cuando aparezca **`jetbot@jetson-desktop`**, estás
conectado. Si ya aparece, salta este paso.

> **Antes de arrancar:** coloca el robot en el cuadrado de **1 × 1 m**, con su
> centro a **28 cm del borde trasero y del izquierdo**, mirando hacia delante.
> El dibujo va hacia su derecha. Para el modo 1, deja el recorrido despejado.

## 2️⃣ TERMINAL 1 · Elige UN modo

### Modo 1 · Dibujar el 17

```bash
cd ~/g17_prii3_ws
source ./entorno_robot.sh
ros2 launch g17_prii3_move_jetbot jetbot.launch.py mode:=draw_number
```

### Modo 2 · Pararse ante un obstáculo

```bash
cd ~/g17_prii3_ws
source ./entorno_robot.sh
ros2 launch g17_prii3_move_jetbot jetbot.launch.py mode:=collision_avoidance scan_yaw_offset:=3.14
```

### Modo 3 · Esquivar el obstáculo

```bash
cd ~/g17_prii3_ws
source ./entorno_robot.sh
ros2 launch g17_prii3_move_jetbot jetbot.launch.py mode:=obstacle_avoidance scan_yaw_offset:=3.14
```

> **Deja esta terminal abierta. El robot arranca PAUSADO.**
> El ajuste `3.14` corresponde a la orientación del LiDAR configurada en este robot.

## 3️⃣ TERMINAL 2 · Comprobar y empezar

**Los mismos pasos sirven para los tres modos.** Espera a que arranque la terminal 1.

Primero, prepara el entorno y comprueba los sensores:

```bash
cd ~/g17_prii3_ws
source ./entorno_robot.sh
ros2 run g17_prii3_move_jetbot diagnose_robot --duration 8
```

Espera unos 8 segundos. Si muestra **`"problems": []`**, ejecuta esta orden:

```bash
ros2 service call /continue_drawing std_srvs/srv/Trigger '{}'
```

**`success=True` → orden aceptada: debería empezar a dibujar.**
Si devuelve un error de sensores, corrígelo antes de continuar.

## ⏸ Pausar · TERMINAL 2

```bash
ros2 service call /stop_drawing std_srvs/srv/Trigger '{}'
```

Para reanudar, repite la orden `continue_drawing` del paso 3.

## 🔄 Cambiar de modo o repetir el 17

1. Pausa desde la **terminal 2**.
2. Pulsa **Ctrl+C en la terminal 1**.
3. Vuelve a colocar el robot en su posición inicial.
4. Ejecuta el modo elegido en la **terminal 1** y la orden de empezar en la **2**.

## Si algo falla

| Lo que aparece | Qué hacer |
|---|---|
| `{}~` o caracteres extra al pegar | Pulsa **Ctrl+C** y escribe la línea a mano, sin añadir `~` al final. |
| `waiting for service...` | Comprueba que la terminal 1 sigue ejecutando el modo y ambas están dentro del robot con el entorno cargado. |
| `success=False` | Lee el mensaje: la orden no se ha aceptado. |

**Los comandos de la terminal 2 se escriben donde aparece el prompt, nunca entre los mensajes del lanzamiento de la terminal 1.**
