"""Tres modos de dibujo del 17 para TurtleBot3 Waffle en Gazebo."""

import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .geometry import (front_min, local_to_world, nearest_in_sector,
                       sector_min, wrap_angle)


# Cada punto indica (adelante, izquierda, dibujar, id del trazo), respecto al
# inicio del robot. Los puntos intermedios colocan el robot sin dibujar.
WAYPOINTS = (
    (0.0, -0.9, True, 0),
    (0.6, -0.9, False, None),
    (0.6, 0.0, False, None),
    (1.3, 0.0, True, 1),
    (0.85, -0.9, True, 1),
)


class DrawingController(Node):
    """Sigue el 17 con odometría; opcionalmente usa el LiDAR."""

    waypoints = WAYPOINTS
    odom_topic = '/odom'
    control_period = 0.1
    max_linear = 0.15
    min_linear = 0.035
    creep_linear = 0.02
    max_angular = 0.9
    turn_gain = 1.7
    goal_tolerance = 0.09
    collision_stop_distance = 0.55
    collision_resume_distance = 0.70
    obstacle_trigger_distance = 0.80
    obstacle_emergency_distance = 0.42
    detour_lateral = 1.0
    detour_forward = 0.9

    # Prepara el controlador simulado, sus topics, servicios y temporizador.
    def __init__(self, mode):
        super().__init__(mode)
        self.odom_topic = self.declare_parameter(
            'odom_topic', self.odom_topic).value
        self.mode = mode
        self.pose = None
        self.origin = None
        self.scan = None
        self.scan_received_at = None
        self.odom_received_at = None
        self.waypoint = 0
        self.paused = False
        self.blocked = False
        self.detour = []
        self.detour_side = 1
        self.last_drawn = {0: None, 1: None}
        self.strokes = {0: [], 1: []}
        self.finished_logged = False
        self.last_sensor_warning = 0.0

        # El controlador manda velocidades, lee sensores y publica el dibujo.
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/g17/drawing', 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        if mode != 'draw_number':
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.stop_srv = self.create_service(
            Trigger, '/stop_drawing', self.on_stop)
        self.continue_srv = self.create_service(
            Trigger, '/continue_drawing', self.on_continue)
        self.restart_srv = self.create_service(
            Trigger, '/restart_drawing', self.on_restart)
        self.timer = self.create_timer(self.control_period, self.control)
        self.get_logger().info(
            f'{mode} listo; esperando {self.odom_topic}'
            + (' y /scan' if mode != 'draw_number' else ''))

    # Actualiza la posicion simulada y fija el origen del dibujo.
    def on_odom(self, msg):
        # Convierte la orientacion ROS (cuaternion) a yaw para guiar el robot.
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y ** 2 + rotation.z ** 2),
        )
        self.pose = (position.x, position.y, yaw)
        self.odom_received_at = time.monotonic()
        if self.origin is None:
            # El primer dato de odometria fija el origen del numero.
            self.origin = self.pose
            self.get_logger().info('Odometría recibida; comienza el dibujo')

    # Guarda el barrido LiDAR simulado y su instante de recepcion.
    def on_scan(self, msg):
        self.scan = msg
        self.scan_received_at = time.monotonic()

    # Publica una velocidad nula para detener el robot simulado.
    def publish_stop(self):
        self.velocity_pub.publish(Twist())

    # Atiende el servicio que pausa el dibujo simulado.
    def on_stop(self, _request, response):
        self.paused = True
        self.publish_stop()
        response.success = True
        response.message = 'Dibujo pausado'
        return response

    # Reanuda el dibujo simulado si aun quedan puntos pendientes.
    def on_continue(self, _request, response):
        if self.waypoint >= len(self.waypoints):
            response.success = False
            response.message = 'Dibujo terminado; utiliza /restart_drawing'
        else:
            self.paused = False
            response.success = True
            response.message = 'Dibujo reanudado'
        return response

    # Borra la traza y reinicia el dibujo desde la posicion actual.
    def on_restart(self, _request, response):
        self.publish_stop()
        # En Gazebo no hay lienzo fisico. Se borra la traza y el siguiente 17
        # empieza en la posicion actual del robot, sin mover otros modelos.
        if self.pose is not None:
            self.origin = self.pose
        self.waypoint = 0
        self.paused = False
        self.blocked = False
        self.detour = []
        self.finished_logged = False
        self.last_drawn = {0: None, 1: None}
        self.strokes = {0: [], 1: []}
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        response.success = True
        response.message = (
            'Traza borrada; nuevo dibujo desde la posición actual')
        return response

    # Comprueba que los sensores necesarios tengan datos recientes.
    def sensor_ready(self):
        # Si los datos necesarios estan viejos, el robot espera parado.
        now = time.monotonic()
        odom_ok = self.pose is not None and now - self.odom_received_at < 1.0
        scan_ok = (
            self.mode == 'draw_number' or
            (self.scan is not None and now - self.scan_received_at < 1.0))
        if not odom_ok or not scan_ok:
            self.publish_stop()
            if now - self.last_sensor_warning > 5.0:
                missing = self.odom_topic if not odom_ok else '/scan'
                self.get_logger().warn(
                    f'Esperando datos recientes de {missing}')
                self.last_sensor_warning = now
            return False
        return True

    # Transforma el siguiente punto del dibujo al marco de odometria.
    def target(self):
        forward, left, _, _ = self.waypoints[self.waypoint]
        return local_to_world(self.origin, forward, left)

    # Calcula y publica velocidades para alcanzar el objetivo indicado.
    def drive_to(self, target, keep_moving=False):
        # Gira hacia el objetivo y avanza mas despacio al acercarse.
        x, y, yaw = self.pose
        distance = math.hypot(target[0] - x, target[1] - y)
        if distance < self.goal_tolerance:
            self.publish_stop()
            return True
        wanted = math.atan2(target[1] - y, target[0] - x)
        error = wrap_angle(wanted - yaw)
        velocity = Twist()
        velocity.angular.z = max(
            -self.max_angular,
            min(self.max_angular, self.turn_gain * error))
        if abs(error) < 0.45:
            velocity.linear.x = min(
                self.max_linear,
                max(self.min_linear, 0.6 * distance))
        elif keep_moving:
            velocity.linear.x = self.creep_linear
        self.velocity_pub.publish(velocity)
        return False

    # Guarda el recorrido simulado y publica su representacion para RViz.
    def record_stroke(self):
        # Guarda posiciones del robot y las publica como una linea para RViz.
        _, _, drawing, stroke_id = self.waypoints[self.waypoint]
        if not drawing:
            return
        x, y, _ = self.pose
        previous = self.last_drawn[stroke_id]
        if (previous is not None and
                math.hypot(x - previous[0], y - previous[1]) < 0.025):
            return
        self.last_drawn[stroke_id] = (x, y)
        self.strokes[stroke_id].append((x, y))
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'numero_17'
        marker.id = stroke_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.color.a = 1.0
        marker.color.r = 0.1 if stroke_id == 0 else 1.0
        marker.color.g = 0.5
        marker.color.b = 1.0 if stroke_id == 0 else 0.1
        marker.points = [Point(x=px, y=py, z=0.06)
                         for px, py in self.strokes[stroke_id]]
        self.marker_pub.publish(marker)

    # Elige el lado mas despejado y prepara dos puntos para rodear el objeto.
    def create_detour(self):
        """Elige el lado más despejado y crea dos objetivos para rodear."""
        x, y, _ = self.pose
        goal = self.target()
        course = math.atan2(goal[1] - y, goal[0] - x)
        left = sector_min(self.scan, 30, 110)
        right = sector_min(self.scan, -110, -30)
        self.detour_side = 1 if left >= right else -1
        lateral = self.detour_side * self.detour_lateral
        side_target = (
            x + 0.12 * math.cos(course) - lateral * math.sin(course),
            y + 0.12 * math.sin(course) + lateral * math.cos(course),
        )
        ahead_target = (
            side_target[0] + self.detour_forward * math.cos(course),
            side_target[1] + self.detour_forward * math.sin(course),
        )
        self.detour = [side_target, ahead_target]
        side = 'izquierda' if lateral > 0 else 'derecha'
        self.get_logger().info('Obstáculo: rodeando por ' + side)

    # Gestiona la pausa, los obstaculos y el avance del dibujo simulado.
    def control(self):
        # Bucle principal: comprueba sensores, aplica el modo y avanza puntos.
        if not self.sensor_ready():
            return
        if self.paused:
            self.publish_stop()
            return
        if self.waypoint >= len(self.waypoints):
            self.publish_stop()
            if not self.finished_logged:
                self.get_logger().info('Dibujo 17 terminado')
                self.finished_logged = True
            return

        self.record_stroke()

        if self.mode == 'collision_avoidance':
            # Se detiene cerca del obstaculo y reanuda con mas distancia.
            distance = front_min(self.scan)
            if distance < self.collision_stop_distance:
                if not self.blocked:
                    self.get_logger().info('Obstáculo delante: parada')
                self.blocked = True
            elif self.blocked and distance > self.collision_resume_distance:
                self.blocked = False
                self.get_logger().info('Vía libre: reanudando')
            if self.blocked:
                self.publish_stop()
                return

        if self.mode == 'obstacle_avoidance':
            # Prepara un desvio; a distancia critica, gira para apartarse.
            front = front_min(self.scan)
            if not self.detour and front < self.obstacle_trigger_distance:
                self.create_detour()
            nearby, bearing = nearest_in_sector(self.scan, -100, 100)
            if nearby < self.obstacle_emergency_distance:
                velocity = Twist()
                velocity.linear.x = self.creep_linear
                direction = (self.detour_side if abs(bearing) < 0.1
                             else -math.copysign(1, bearing))
                velocity.angular.z = self.max_angular * direction
                self.velocity_pub.publish(velocity)
                return
            if self.detour:
                if self.drive_to(self.detour[0], keep_moving=True):
                    self.detour.pop(0)
                return

        if self.drive_to(
                self.target(),
                keep_moving=self.mode == 'obstacle_avoidance'):
            self.waypoint += 1
            if self.waypoint < len(self.waypoints):
                self.get_logger().info(
                    f'Segmento {self.waypoint + 1}/{len(self.waypoints)}')


# Ejecuta el controlador simulado y detiene el robot al finalizar.
def run(mode, args=None):
    rclpy.init(args=args)
    node = DrawingController(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# Inicia el dibujo del 17 en simulacion.
def main_draw(args=None):
    run('draw_number', args)


# Inicia la simulacion que espera ante obstaculos.
def main_collision(args=None):
    run('collision_avoidance', args)


# Inicia la simulacion que rodea obstaculos.
def main_obstacle(args=None):
    run('obstacle_avoidance', args)
