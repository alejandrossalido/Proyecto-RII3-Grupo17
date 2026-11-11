# Compilar y ejecutar con Distrobox

[Volver al README](../README.md)

En mi ordenador uso Omarchy y tengo ROS dentro de una caja de Distrobox llamada ros-humble, con Ubuntu 22.04.

## Qué hace falta

- Distrobox y Docker o Podman.
- Una caja con Ubuntu 22.04, ROS 2 Humble, Python 3, colcon y turtlesim.
- El proyecto dentro de una carpeta accesible desde la caja. En mi instalación se comparte la carpeta de usuario.

Estos pasos parten de que la caja ya está preparada. Si acabas de crearla, instala ROS y las herramientas dentro siguiendo la [guía de Ubuntu](ubuntu.md).

## Entrar y ejecutar

Desde una terminal normal del ordenador:

    distrobox enter --clean-path --name ros-humble

Uso --clean-path para que se utilice el Python de Ubuntu dentro de la caja. Si tu caja tiene otro nombre, cambia ros-humble.

Ya dentro, entra en la copia descargada:

    cd ~/Projects/Sprint-1-PRII

Si usas mi copia original, la ruta es esta. Usa solo uno de los dos comandos cd:

    cd "/home/alejandrossalido/Projects/01_Universidad/3º_de_Carrera/Proyecto_RII_3/Sprint1/g17_PRII3_ws"

Después compila y arranca el proyecto:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select g17_prii3_turtlesim
    source install/setup.bash
    ros2 launch g17_prii3_turtlesim draw_17.launch.py

Los comandos se ejecutan dentro del Ubuntu de la caja y la ventana de turtlesim aparece en el escritorio.

Deja esa terminal abierta. Para cerrar el programa, pulsa Ctrl+C. Si no has cambiado el código, puedes saltarte colcon build, pero carga el entorno en cada terminal nueva.

Los comandos para parar, reanudar y reiniciar están en la [guía de servicios](servicios.md).
