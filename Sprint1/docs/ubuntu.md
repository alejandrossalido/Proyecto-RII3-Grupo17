# Compilar y ejecutar en Ubuntu

[Volver al README](../README.md)

## Qué hace falta

- Ubuntu 22.04 con escritorio.
- ROS 2 Humble instalado. La opción Desktop incluye las herramientas habituales.
- Python 3, que ya viene con Ubuntu.
- colcon y turtlesim.

Si te falta ROS, sigue la [guía de instalación de Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Con ROS instalado y sus repositorios configurados, instala las herramientas que falten:

    sudo apt update
    sudo apt install python3-colcon-common-extensions ros-humble-turtlesim

## Compilar y arrancar

Si has seguido la [guía de descarga](descargar.md), entra en esta carpeta:

    cd ~/Projects/Sprint-1-PRII

Si lo tienes en otro sitio, entra en la carpeta que contiene src y el README. En mi copia original se llama g17_PRII3_ws.

Ejecuta estos comandos en orden:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select g17_prii3_turtlesim
    source install/setup.bash
    ros2 launch g17_prii3_turtlesim draw_17.launch.py

El primer source carga ROS. colcon prepara nuestro paquete y el segundo source permite que ROS lo encuentre. El último comando abre el simulador y arranca el dibujo del 17.

El launch ya inicia turtlesim. Deja esa terminal abierta mientras funciona y pulsa Ctrl+C para terminar.

En cada terminal nueva hay que repetir los comandos source. Si no has cambiado el código, puedes saltarte colcon build.

Para parar, reanudar o reiniciar el dibujo, sigue la [guía de servicios](servicios.md).
