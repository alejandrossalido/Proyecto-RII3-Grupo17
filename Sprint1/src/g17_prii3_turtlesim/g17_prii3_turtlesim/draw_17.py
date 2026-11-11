"""Nodo ROS 2 que dibuja automáticamente el número 17 en turtlesim."""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, Trigger
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class Draw17(Node):
    """Controla la tortuga y expone servicios para pausar y reiniciar."""

    def __init__(self):
        """Configura las comunicaciones y el control del dibujo."""
        super().__init__('draw_17')

        # Comunicaciones ROS 2
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Clientes de los servicios de turtlesim
        self.pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen'
        )

        self.reset_client = self.create_client(
            Empty,
            '/reset'
        )

        # Espera a que turtlesim esté listo para recibir peticiones.
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Esperando al servicio /turtle1/set_pen...'
            )

        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /reset...')

        # Estado y secuencia del dibujo
        self.pose = None
        self.actions = [
            # 'pen' cambia el trazo y 'move' define un destino (x, y).
            ('pen', False),
            ('move', 3.0, 8.0),
            ('pen', True),
            ('move', 3.0, 3.0),
            ('pen', False),
            ('move', 5.0, 8.0),
            ('pen', True),
            ('move', 8.0, 8.0),
            ('move', 5.5, 3.0),
            ('pen', False),
        ]

        # Índice, peticiones asíncronas y pausa actual.
        self.current_action = 0
        self.pen_future = None
        self.reset_future = None
        self.paused = False

        # Servicios de control del dibujo
        self.stop_service = self.create_service(
            Trigger,
            '/stop_drawing',
            self.stop_drawing_callback
        )
        self.continue_service = self.create_service(
            Trigger,
            '/continue_drawing',
            self.continue_drawing_callback
        )
        self.restart_service = self.create_service(
            Trigger,
            '/restart_drawing',
            self.restart_drawing_callback
        )

        # Ejecuta el control a 10 Hz.
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('Nodo draw_17 iniciado')

    def pose_callback(self, msg):
        """Actualiza la posición actual de turtle1."""
        self.pose = msg

    def set_pen(self, enabled):
        """Activa o desactiva el lápiz de turtle1."""
        request = SetPen.Request()
        request.r = 0
        request.g = 0
        request.b = 255
        request.width = 5
        # En SetPen, off=1 levanta el lápiz y off=0 dibuja.
        request.off = 0 if enabled else 1

        self.pen_future = self.pen_client.call_async(request)

    def stop_turtle(self):
        """Publica velocidad cero para detener la tortuga."""
        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def stop_drawing_callback(self, request, response):
        """Pausa el dibujo manteniendo la acción actual."""
        del request
        self.paused = True
        self.stop_turtle()
        response.success = True
        response.message = 'Dibujo pausado'
        self.get_logger().info(response.message)
        return response

    def continue_drawing_callback(self, request, response):
        """Reanuda el dibujo desde el punto en que se detuvo."""
        del request

        if self.reset_future is not None:
            response.success = False
            response.message = 'El reinicio está en curso'
            return response

        if self.current_action >= len(self.actions):
            response.success = False
            response.message = 'El dibujo ya ha terminado; usa /restart_drawing'
            return response

        self.paused = False
        response.success = True
        response.message = 'Dibujo reanudado'
        self.get_logger().info(response.message)
        return response

    def restart_drawing_callback(self, request, response):
        """Borra turtlesim y vuelve a iniciar la secuencia completa."""
        del request

        if self.reset_future is not None:
            response.success = False
            response.message = 'Ya hay un reinicio en curso'
            return response

        self.paused = True
        self.pen_future = None
        self.stop_turtle()
        self.reset_future = self.reset_client.call_async(Empty.Request())

        response.success = True
        response.message = 'Reinicio solicitado'
        self.get_logger().info(response.message)
        return response

    def move_to(self, target_x, target_y):
        """Mueve la tortuga al destino y devuelve True al llegar."""
        # Calcula el error de posición y orientación.
        distance = math.sqrt(
            (target_x - self.pose.x) ** 2 +
            (target_y - self.pose.y) ** 2
        )

        target_angle = math.atan2(
            target_y - self.pose.y,
            target_x - self.pose.x
        )

        angle_error = target_angle - self.pose.theta
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # Da por alcanzado el punto dentro del margen establecido.
        if distance < 0.1:
            self.stop_turtle()
            return True

        msg = Twist()

        # Gira primero si el error angular es grande; si no, avanza y corrige.
        if abs(angle_error) > 0.15:
            msg.linear.x = 0.0
            msg.angular.z = 2.0 * angle_error

        else:
            msg.linear.x = min(1.5, 1.5 * distance)
            msg.angular.z = 2.0 * angle_error

        self.cmd_vel_pub.publish(msg)

        return False

    def control_loop(self):
        """Actualiza el dibujo en cada ciclo del temporizador."""
        # No controla el movimiento hasta recibir la primera pose.
        if self.pose is None:
            return

        # Espera a que el servicio de reinicio termine antes de continuar.
        if self.reset_future is not None:
            self.stop_turtle()

            if self.reset_future.done():
                try:
                    self.reset_future.result()
                except Exception as error:
                    self.get_logger().error(
                        f'No se pudo reiniciar turtlesim: {error}'
                    )
                    self.paused = True
                else:
                    self.current_action = 0
                    self.paused = False
                    self.get_logger().info('Dibujo reiniciado')

                self.reset_future = None

            return

        # La pausa conserva la acción actual para poder reanudarla.
        if self.paused:
            self.stop_turtle()
            return

        if self.current_action >= len(self.actions):
            self.stop_turtle()
            return

        # Espera la respuesta del último cambio de lápiz.
        if self.pen_future is not None:

            self.stop_turtle()

            if self.pen_future.done():
                self.pen_future = None
                self.current_action += 1

            return

        action = self.actions[self.current_action]

        # Ejecuta la acción actual: cambiar el lápiz o moverse al destino.
        if action[0] == 'pen':

            enabled = action[1]

            if enabled:
                self.get_logger().info('Bajando lápiz')
            else:
                self.get_logger().info('Levantando lápiz')

            self.set_pen(enabled)

        elif action[0] == 'move':

            target_x = action[1]
            target_y = action[2]

            arrived = self.move_to(
                target_x,
                target_y
            )

            if arrived:
                self.get_logger().info(
                    f'Objetivo alcanzado: ({target_x}, {target_y})'
                )

                self.current_action += 1


def main(args=None):
    """Inicializa y ejecuta el nodo de dibujo."""
    # Inicio y ciclo de vida del nodo ROS 2.
    rclpy.init(args=args)
    node = Draw17()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Cierre controlado.
    node.stop_turtle()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
