# Parar, reanudar y reiniciar la tortuga

[Volver al README](../README.md)

## Preparar otra terminal

Deja abierta la terminal del launch y abre una segunda. Si usas Distrobox, entra también en la caja:

    distrobox enter --clean-path --name ros-humble

En Ubuntu directamente, sáltate ese paso. Entra en la misma carpeta desde la que arrancaste el proyecto y carga el entorno:

    cd ~/Projects/Sprint-1-PRII
    source /opt/ros/humble/setup.bash
    source install/setup.bash

Si usas mi copia original, cambia el cd por la ruta de g17_PRII3_ws que aparece en la [guía de Distrobox](distrobox.md).

## Parar

Ejecuta este comando mientras la tortuga está dibujando:

    ros2 service call /stop_drawing std_srvs/srv/Trigger "{}"

Se queda quieta y conserva el punto del dibujo. La respuesta debe mostrar success=True y "Dibujo pausado".

## Reanudar

Para continuar desde donde se quedó:

    ros2 service call /continue_drawing std_srvs/srv/Trigger "{}"

La respuesta debe mostrar success=True y "Dibujo reanudado". Si el dibujo ya terminó, usa el reinicio.

## Reiniciar

Para borrar el lienzo y empezar de nuevo:

    ros2 service call /restart_drawing std_srvs/srv/Trigger "{}"

La respuesta debe mostrar success=True y "Reinicio solicitado". La tortuga vuelve a la posición inicial y comienza otra vez.

Para enseñarlo en clase, puedes pausar mientras se mueve, reanudarla y después reiniciarla. Ejecuta los comandos de uno en uno.

## Comprobar que funciona

Desde esta segunda terminal, con el entorno cargado:

    ros2 node list
    ros2 topic list
    ros2 service list

Deberían aparecer los nodos /draw_17 y /turtlesim. Nuestro nodo envía velocidades por /turtle1/cmd_vel y recibe la posición por /turtle1/pose.

Para cerrar el programa, pulsa Ctrl+C en la primera terminal. stop_drawing solo pausa el dibujo y deja los nodos abiertos para poder reanudarlo.
