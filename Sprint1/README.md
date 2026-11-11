# Sprint 1 PRII3 - Grupo 17

En este proyecto usamos ROS 2 y turtlesim para dibujar el número 17. Nuestro nodo en Python controla el movimiento y tiene servicios para parar, reanudar y reiniciar el dibujo.

## Cómo usarlo

Hemos separado las instrucciones para que sea más fácil encontrar cada cosa:

- [Descargar el proyecto](docs/descargar.md): hacer una copia de GitHub y actualizarla.
- [Compilar y ejecutar en Ubuntu](docs/ubuntu.md): requisitos y comandos con ROS 2 Humble.
- [Compilar y ejecutar con Distrobox](docs/distrobox.md): como lo tengo preparado en Omarchy.
- [Controlar la tortuga](docs/servicios.md): parar, reanudar, reiniciar y comprobar los nodos.

Si empiezas de cero, descarga el proyecto y después sigue la guía de Ubuntu o la de Distrobox, según tu ordenador.

## Archivos principales

Dentro de src/g17_prii3_turtlesim están:

- g17_prii3_turtlesim/draw_17.py: movimiento, lápiz y servicios del dibujo.
- launch/draw_17.launch.py: arranca turtlesim y nuestro nodo juntos.
- setup.py y setup.cfg: instalación del paquete.
- package.xml: información y dependencias.
