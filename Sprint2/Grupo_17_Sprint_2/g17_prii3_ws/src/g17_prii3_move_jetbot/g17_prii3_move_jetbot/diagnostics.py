"""Diagnóstico pasivo de ROS: nunca publica velocidades ni llama servicios."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .motion import scan_sector


# Resume los nodos, tipos de mensaje y QoS de cada conexion ROS.
def endpoints(items):
    return [dict(node=f'{item.node_namespace}/{item.node_name}'.replace('//', '/'),
                 type=item.topic_type, reliability=str(item.qos_profile.reliability),
                 durability=str(item.qos_profile.durability)) for item in items]


class Diagnostic(Node):
    # Crea el nodo de diagnostico y las suscripciones a los sensores.
    def __init__(self, args):
        super().__init__('g17_diagnose_robot')
        self.args = args
        self.samples = {'odom': [], 'scan': []}
        self.latest = {}
        self.subscriptions_kept = [
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data),
            self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)]

    # Registra la recepcion del mensaje y calcula su antigüedad.
    def common(self, kind, msg):
        self.samples[kind].append(time.monotonic())
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        return {'stamp': stamp, 'age_s_at_receive': self.get_clock().now().nanoseconds*1e-9-stamp,
                'frame': msg.header.frame_id}

    # Recoge posicion, orientacion y velocidad para revisar la odometria.
    def on_odom(self, msg):
        data = self.common('odom', msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        data.update(position_m=[p.x, p.y], quaternion=[q.x, q.y, q.z, q.w],
                    quaternion_norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w),
                    child_frame=msg.child_frame_id,
                    linear_m_s=msg.twist.twist.linear.x, angular_rad_s=msg.twist.twist.angular.z)
        self.latest['odom'] = data

    # Recoge las lecturas LiDAR y calcula la distancia del sector frontal.
    def on_scan(self, msg):
        data = self.common('scan', msg)
        usable = sum(v == math.inf or v == -math.inf or
                     (math.isfinite(v) and v > 0 and v <= msg.range_max) for v in msg.ranges)
        front = scan_sector(msg, -math.pi/6, math.pi/6)
        data.update(rays=len(msg.ranges), usable_rays=usable, frame=msg.header.frame_id,
                    front_m=front, range_min=msg.range_min, range_max=msg.range_max)
        self.latest['scan'] = data

    # Genera el informe de conexiones, frecuencias y problemas detectados.
    def report(self):
        topics = dict(self.get_topic_names_and_types())
        graph = {}
        for topic in (self.args.odom_topic, self.args.scan_topic, self.args.cmd_vel_topic):
            graph[topic] = dict(types=topics.get(topic, []),
                                publishers=endpoints(self.get_publishers_info_by_topic(topic)),
                                subscribers=endpoints(self.get_subscriptions_info_by_topic(topic)))
        problems = []
        for kind, topic in (('odom', self.args.odom_topic), ('scan', self.args.scan_topic)):
            samples = self.samples[kind]
            rate = (len(samples)-1)/(samples[-1]-samples[0]) if len(samples) > 1 else 0
            last = self.latest.get(kind, {})
            last.update(messages=len(samples), rate_hz=rate,
                        seconds_since_last=time.monotonic()-samples[-1] if samples else None)
            self.latest[kind] = last
            if len(samples) < 2:
                problems.append(f'No llegan datos suficientes de {topic}; buscar tipo, nombre y QoS reales')
            elif last['seconds_since_last'] > 0.6 or not -0.1 <= last['age_s_at_receive'] <= 0.6:
                problems.append(f'{topic}: datos antiguos o reloj sin sincronizar')
            if len(samples) >= 2 and rate < (10 if kind == 'odom' else 3):
                problems.append(f'{topic}: frecuencia baja ({rate:.1f} Hz); revisar pérdidas o driver lento')
            if len(graph[topic]['publishers']) > 1:
                problems.append(f'{topic}: varios publicadores; posible mezcla de robots o chasis duplicado')
        cmd = graph[self.args.cmd_vel_topic]
        if len(cmd['publishers']) > 1:
            problems.append('Varios publicadores cmd_vel; cerrar teleop y otros controladores')
        if not cmd['subscribers']:
            problems.append('No hay suscriptor de cmd_vel; chasis apagado, tipo incorrecto o tema equivocado')
        if cmd['types'] and cmd['types'] != ['geometry_msgs/msg/Twist']:
            problems.append('cmd_vel no usa exclusivamente Twist; revisar el contrato del driver')
        odom = self.latest['odom']
        if any(not math.isfinite(v) for v in odom.get('position_m', [])):
            problems.append('Posición de odometría no finita')
        if 'quaternion_norm' in odom and (not math.isfinite(odom['quaternion_norm'])
                                         or abs(odom['quaternion_norm']-1) > 0.1):
            problems.append('Cuaternión de odometría incorrecto')
        if self.latest['scan'].get('front_m') is None:
            problems.append('LiDAR sin rayos válidos en el sector frontal')
        import os
        return dict(passive=True, environment={key: os.environ.get(key) for key in (
            'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')},
            loaded_module=str(Path(__file__).resolve()), graph=graph, sensors=self.latest,
            discovered_topics=topics, problems=problems,
            limitation='Recibir datos no demuestra exactitud física de encoders, IMU o LiDAR. No se han movido motores.')


# Convierte valores no finitos para guardar el informe en JSON.
def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


# Escucha los sensores durante el tiempo indicado y guarda el diagnostico.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--odom-topic', default='/wheel/odometry')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--output', default='diagnostico_robot.json')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error('--duration debe estar entre 1 y 60 segundos')
    rclpy.init(args=ros_args)
    node = Diagnostic(args)
    try:
        until = time.monotonic()+args.duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = json_safe(node.report())
        Path(args.output).write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
