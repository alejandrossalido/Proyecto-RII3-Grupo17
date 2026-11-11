"""Control del JetBot real. El controlador de Gazebo permanece independiente."""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .motion import (Config, SegmentFollower, START, WAYPOINTS, Workspace,
                     path_clear, plan_detour, scan_points, scan_sector, wrap)


class JetbotDrawingController(Node):
    """Un único controlador para un único chasis, inicialmente pausado."""

    # Prepara el nodo ROS, los parametros, los topics y los servicios del modo.
    def __init__(self, mode):
        super().__init__('g17_' + mode)
        self.mode = mode
        self.config = Config(**{
            key: self.declare_parameter(key, value).value
            for key, value in Config().__dict__.items()})
        self.odom_topic = self.declare_parameter('odom_topic', '/wheel/odometry').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        cmd_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        if mode not in ('draw_number', 'collision_avoidance', 'obstacle_avoidance'):
            raise ValueError('Modo de dibujo desconocido')
        self.require_scan = mode != 'draw_number'
        self.scan_yaw = self.declare_parameter('scan_yaw_offset', 0.0).value
        self.scan_x = self.declare_parameter('scan_x', 0.0).value
        self.scan_y = self.declare_parameter('scan_y', 0.0).value
        if not all(math.isfinite(v) for v in (self.scan_yaw, self.scan_x, self.scan_y)):
            raise ValueError('Extrínsecos del LiDAR no válidos')
        self.paused = self.declare_parameter('start_paused', True).value
        self.workspace = Workspace(self.config)
        self.follower = SegmentFollower(self.config)
        self.pose = self.raw_pose = None
        self.scan = None
        self.odom_at = self.scan_at = None
        self.odom_stamp = self.scan_stamp = None
        self.odom_frames = None
        self.frame = 'odom'
        self.speed = 0.0
        self.reset_drawing()
        self.last_tick = time.monotonic()
        self.settle_until = 0.0
        self.graph_at = 0.0
        self.graph_ok = False
        self.last_warning = 0.0
        # Solo la orden mas reciente; no acumular diez ordenes antiguas.
        self.velocity_pub = self.create_publisher(Twist, cmd_topic, QoSProfile(depth=1))
        self.marker_pub = self.create_publisher(Marker, '/g17/drawing', 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        if self.require_scan:
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        for action in ('stop', 'continue', 'restart'):
            self.create_service(Trigger, f'/{action}_drawing', getattr(self, f'on_{action}'))
        self.timer = self.create_timer(0.05, self.control)
        self.get_logger().info(
            f'{mode}: recinto fijo 1 × 1 m, ruta 0,44 × 0,305 m; '
            f'odom={self.odom_topic}; pausado={self.paused}. '
            'Centro inicial a 28 cm del borde trasero y 28 cm del izquierdo.')

    # Restablece el progreso del dibujo conservando la referencia del recinto.
    def reset_drawing(self):
        """Estado común al iniciar y al reiniciar. El recinto permanece fijo."""
        self.fault = None
        self.was_running = False
        self.waypoint = 0
        self.detour = []
        self.detour_goal = 0
        self.replan_at = 0.0
        self.leg_start = START
        self.blocked = False
        self.leg_active_time = 0.0
        self.progress_pose = self.pose
        self.last_progress_at = time.monotonic()
        self.strokes = {0: [], 1: []}
        self.follower.reset()

    # Envia velocidad cero al chasis y detiene el seguidor.
    def publish_stop(self):
        self.follower.stop()
        self.velocity_pub.publish(Twist())

    # Registra un fallo y deja el robot detenido y pausado.
    def fail(self, reason):
        if self.fault is None:
            self.get_logger().error(reason + '; parada enclavada')
        self.fault = reason
        self.paused = True
        self.publish_stop()

    # Rechaza mensajes antiguos, repetidos o con retrocesos de reloj.
    def valid_stamp(self, stamp, previous, max_age):
        value = stamp.sec + stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if value <= 0 or not -0.10 <= now - value <= max_age:
            return None  # Impide aceptar una cola de datos antiguos como reciente.
        if previous is not None and value < previous:
            self.fail('El reloj del sensor retrocede o se reinició su nodo')
            return None
        if previous is not None and value == previous:
            return None
        return value

    # Valida la odometria recibida y actualiza la posicion del robot.
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w,
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not all(math.isfinite(v) for v in values) or not 0.9 < norm < 1.1:
            self.fail('Odometría no válida (NaN o cuaternión incorrecto)')
            return
        stamp = self.valid_stamp(msg.header.stamp, self.odom_stamp, self.config.sensor_timeout)
        if stamp is None:
            return
        frames = (msg.header.frame_id, msg.child_frame_id)
        if not all(frames) or (self.odom_frames is not None and frames != self.odom_frames):
            self.fail('Marcos de odometría vacíos o cambiantes')
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y)/norm**2,
                         1-2*(q.y*q.y+q.z*q.z)/norm**2)
        raw = (p.x, p.y, yaw)
        if self.raw_pose is not None:
            dt = stamp - self.odom_stamp
            distance = math.hypot(raw[0]-self.raw_pose[0], raw[1]-self.raw_pose[1])
            angle = abs(wrap(raw[2]-self.raw_pose[2]))
            if distance*self.config.linear_scale > 0.4*dt+0.02 or angle > 1.5*dt+0.08:
                self.fail('Salto de odometría o movimiento superior al esperado')
                return
        self.odom_frames, self.frame = frames, frames[0]
        self.raw_pose, self.odom_stamp = raw, stamp
        self.pose = self.workspace.pose(raw)
        self.speed = abs(msg.twist.twist.linear.x)*self.config.linear_scale
        self.odom_at = time.monotonic()

    # Guarda el ultimo barrido LiDAR valido y su instante de recepcion.
    def on_scan(self, msg):
        stamp = self.valid_stamp(msg.header.stamp, self.scan_stamp, self.config.scan_timeout)
        if stamp is None:
            return
        if (not msg.ranges or not math.isfinite(msg.angle_min) or
                not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max) or
                not 0 <= msg.range_min < msg.range_max):
            return
        self.scan, self.scan_stamp, self.scan_at = msg, stamp, time.monotonic()

    # Calcula la distancia minima de un sector respecto al cuerpo del robot.
    def sector(self, start, end):
        return scan_sector(self.scan, math.radians(start), math.radians(end),
                           self.scan_yaw, self.scan_x, self.scan_y)

    # Indica que sensor o conexion impide continuar, si falta alguno.
    def ready_reason(self):
        now = time.monotonic()
        if self.odom_at is None or now-self.odom_at > self.config.sensor_timeout:
            return f'Sin odometría reciente en {self.odom_topic}; revisar tema/reloj/QoS'
        if self.require_scan and (self.scan_at is None or now-self.scan_at > self.config.scan_timeout):
            return f'Sin LiDAR reciente en {self.scan_topic}; revisar tema/reloj/QoS'
        if self.require_scan and self.sector(-30, 30) is None:
            return 'LiDAR sin lecturas válidas en el sector frontal'
        if self.mode == 'obstacle_avoidance' and any(
                self.sector(start, start+90) is None for start in (-180, -90, 0, 90)):
            return 'El rodeo necesita lecturas LiDAR delante, detrás y a ambos lados'
        if not self.graph_ok:
            return 'Esperando un chasis y un único origen por sensor'
        return None

    # Comprueba las conexiones ROS y detecta publicadores duplicados.
    def check_graph(self):
        now = time.monotonic()
        if now < self.graph_at:
            return
        self.graph_at = now + 0.5
        publishers = self.count_publishers(self.velocity_pub.topic_name)
        odom_sources = self.count_publishers(self.odom_sub.topic_name)
        scan_sources = self.count_publishers(self.scan_sub.topic_name) if self.require_scan else 1
        if publishers > 1 or odom_sources > 1 or scan_sources > 1:
            self.fail('Publicadores duplicados de velocidad/odometría/LiDAR; aislar robot y cerrar otros controles')
        self.graph_ok = (publishers == 1 and odom_sources == 1 and scan_sources == 1
                         and self.velocity_pub.get_subscription_count() > 0)

    # Atiende el servicio de pausa y ordena detener el robot.
    def on_stop(self, _request, response):
        self.paused = True
        self.publish_stop()
        response.success, response.message = True, 'Pausado'
        return response

    # Atiende el servicio de continuar si los sensores y el estado lo permiten.
    def on_continue(self, _request, response):
        self.check_graph()
        reason = self.fault or self.ready_reason()
        if self.waypoint >= len(WAYPOINTS):
            reason = 'Dibujo terminado; recolocar en el inicio y usar restart'
        response.success = reason is None
        response.message = reason or 'Reanudado en el mismo recinto'
        if response.success:
            self.paused = False
            self.progress_pose = self.pose
            self.last_progress_at = time.monotonic()
        return response

    # Reinicia el dibujo cerca de su origen y lo deja pausado.
    def on_restart(self, _request, response):
        self.publish_stop()
        self.paused = True
        self.check_graph()
        reason = self.ready_reason()
        if reason is None and (math.hypot(self.pose[0]-START[0], self.pose[1]-START[1]) > 0.025
                               or abs(self.pose[2]) > math.radians(5)):
            reason = 'Recolocar en el inicio y orientación originales; el recinto nunca se desplaza'
        if reason:
            response.success, response.message = False, reason
            return response
        self.reset_drawing()
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        response.success, response.message = True, 'Reiniciado y pausado; usar continue para mover'
        return response

    # Guarda el trazo estimado por odometria y lo publica para RViz.
    def record_stroke(self):
        _, _, drawing, stroke = WAYPOINTS[self.waypoint]
        if not drawing:
            return
        point = self.raw_pose[:2]
        points = self.strokes[stroke]
        if points and math.hypot(point[0]-points[-1][0], point[1]-points[-1][1]) < 0.005:
            return
        points.append(point)
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'numero_17_fisico', stroke
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.a, marker.color.g = 1.0, 0.6
        marker.color.r, marker.color.b = (0.1, 1.0) if stroke == 0 else (1.0, 0.1)
        marker.points = [Point(x=x, y=y, z=0.03) for x, y in points]
        self.marker_pub.publish(marker)

    # Detiene el robot mientras el obstaculo bloquea el recorrido.
    def wait_for_obstacle(self, now, message):
        """Esperar no borra el dibujo, no pausa el servicio ni cambia el origen."""
        self.blocked = True
        self.publish_stop()
        self.last_progress_at = now
        if now-self.last_warning > 5:
            self.get_logger().warn(message)
            self.last_warning = now
        return True

    # Decide si debe seguir, esperar o calcular un rodeo segun el modo.
    def obstacle_blocks(self, now):
        """Modo 1: ignorar LiDAR. Modo 2: esperar. Modo 3: buscar un rodeo."""
        if not self.require_scan:
            return False
        pad = self.config.robot_radius + self.config.safety_margin
        if self.sector(-180, 180) < pad:
            return self.wait_for_obstacle(now, 'Objeto demasiado cerca del chasis; esperando')
        obstacles = scan_points(self.scan, self.pose, self.scan_yaw, self.scan_x, self.scan_y)
        target = self.detour[0] if self.detour else WAYPOINTS[self.waypoint][:2]
        if path_clear(self.pose, target, obstacles, pad+self.config.goal_tolerance):
            self.blocked = False
            return False
        if self.mode == 'collision_avoidance':
            return self.wait_for_obstacle(now, 'Obstáculo en el trazo; continuaré cuando se retire')
        if now >= self.replan_at:
            self.publish_stop()  # Calcular el rodeo siempre con los motores parados.
            self.replan_at = now+0.5
            # Intentar volver al vertice pendiente. Si esta ocupado, probar los
            # siguientes del MISMO 17. Nunca retroceder al primer vertice.
            for index in range(self.waypoint, len(WAYPOINTS)):
                route = plan_detour(self.workspace, self.pose, WAYPOINTS[index][:2], obstacles)
                if route:
                    self.detour, self.detour_goal = route, index
                    self.leg_start = self.pose[:2]
                    self.leg_active_time = 0.0
                    self.follower.reset()
                    self.blocked = False
                    self.get_logger().info(f'Rodeo; retomaré el mismo 17 en el vértice {index+1}')
                    return True  # Leer sensores de nuevo antes de mover por el rodeo.
        return self.wait_for_obstacle(now, 'No cabe un rodeo dentro de 1 m²; esperando espacio')

    # Avanza al siguiente punto del rodeo o del dibujo original.
    def advance_target(self, target, now):
        """Consumir un punto del rodeo o avanzar al siguiente vértice original."""
        self.publish_stop()
        self.settle_until = now+0.25
        self.leg_start = target
        self.leg_active_time = 0.0
        self.follower.reset()
        if self.detour:
            self.detour.pop(0)
            if self.detour:
                return
            self.waypoint = self.detour_goal
        self.waypoint += 1
        self.get_logger().info(f'Segmento terminado: {self.waypoint}/{len(WAYPOINTS)}')

    # Comprueba frenado y limites antes de publicar las velocidades.
    def send_command(self, v, w, dt, now):
        """Aplicar frenado, límites del recinto y vigilancia de motores."""
        if self.require_scan and v > 0:
            speed = max(self.speed, v)
            stop = (self.config.robot_radius+self.config.safety_margin+0.01
                    + speed*(self.config.scan_timeout+0.05)
                    + speed**2/(2*self.config.linear_accel))
            if self.sector(-30, 30) < stop:
                self.wait_for_obstacle(now, 'Esperando distancia suficiente para frenar')
                return
        self.leg_active_time += min(dt, 0.1)
        if self.leg_active_time > self.config.waypoint_timeout:
            self.fail('Tiempo de tramo agotado; revisar bloqueo, rozamiento y odometría')
            return
        if self.progress_pose is None or (
                math.hypot(self.pose[0]-self.progress_pose[0], self.pose[1]-self.progress_pose[1]) > 0.006
                or abs(wrap(self.pose[2]-self.progress_pose[2])) > 0.04):
            self.progress_pose, self.last_progress_at = self.pose, now
        elif now-self.last_progress_at > 4.0:
            self.fail('Hay órdenes pero la odometría no avanza; revisar motores/encoders/velocidad mínima')
            return
        if not self.workspace.permits(self.pose, v, self.speed):
            self.fail('Margen insuficiente para frenar dentro del cuadrado')
            return
        command = Twist()
        command.linear.x, command.angular.z = v, w
        self.velocity_pub.publish(command)

    # Coordina sensores, obstaculos y seguimiento del recorrido a 20 Hz.
    def control(self):
        """Bucle a 20 Hz: comprobar, gestionar obstáculo y seguir el punto pendiente."""
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        self.check_graph()
        reason = self.ready_reason()
        if reason:
            if self.was_running and not self.paused:
                self.fail(reason)
            else:
                self.publish_stop()
                if now-self.last_warning > 5:
                    self.get_logger().warn(reason)
                    self.last_warning = now
            return
        if self.fault or self.paused or self.waypoint >= len(WAYPOINTS):
            self.publish_stop()
            return
        self.was_running = True
        if not self.workspace.contains(self.pose):
            self.fail('El chasis alcanza el margen del recinto de 1 × 1 m')
            return
        if now < self.settle_until:
            self.publish_stop()
            self.last_progress_at = now
            return
        if self.obstacle_blocks(now):
            return
        self.record_stroke()
        target = self.detour[0] if self.detour else WAYPOINTS[self.waypoint][:2]
        # El rodeo se recorre mas despacio para disponer de margen en 1 m².
        limit = self.config.min_linear if self.detour else self.config.max_linear
        v, w, reached, fault = self.follower.command(
            self.pose, self.leg_start, target, dt, speed_limit=limit)
        if fault:
            self.fail(fault)
        elif reached:
            self.advance_target(target, now)
        else:
            self.send_command(v, w, dt, now)


# Ejecuta el nodo y envia la orden de parada antes de cerrar ROS.
def run(mode, args=None):
    # Mantener vivo DDS hasta enviar los ceros. El manejador automatico de
    # Humble puede cerrar el contexto antes de ejecutar el bloque finally.
    try:
        from rclpy.signals import SignalHandlerOptions
    except ImportError:  # Compatibilidad con instalaciones ROS anteriores.
        rclpy.init(args=args)
    else:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_int = signal.signal(signal.SIGINT, signal.default_int_handler)
    previous_term = signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = JetbotDrawingController(mode)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch puede reenviar el mismo SIGINT; no interrumpir la parada.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.paused = True
            if rclpy.ok():
                for _ in range(3):
                    node.publish_stop()
                    rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)


# Inicia el modo de dibujo sin deteccion de obstaculos.
def main_draw(args=None):
    run('draw_number', args)


# Inicia el modo que espera ante obstaculos y despues continua.
def main_collision(args=None):
    run('collision_avoidance', args)


# Inicia el modo que rodea obstaculos y retoma el dibujo pendiente.
def main_obstacle(args=None):
    run('obstacle_avoidance', args)
