"""Pasillo -> puerta -> ArUco -> destino. La misión avanza mediante callbacks."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .route import MarkerChoice, load_route


class Mission(Node):
    def __init__(self, autonomous=False):
        super().__init__('autonomous_navigation' if autonomous else 'predefined_navigation')
        self.route = load_route(self.declare_parameter('route_file', '').value)
        self.autonomous, self.choice = autonomous, MarkerChoice(self.route['targets'])
        self.nav_timeout = self.declare_parameter('navigation_timeout', 180.0).value
        self.marker_timeout = self.declare_parameter('marker_timeout', 30.0).value
        self.state, self.goal, self.pose = 'idle', None, None
        self.after, self.deadline, self.stop_result = 0.0, 0.0, 'cancelled'
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.nav_active, self.state_request = False, None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/g17/mission/status', qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        self.create_subscription(String, '/g17/aruco_robot', self.on_marker, 1)
        self.create_service(Trigger, '/g17/mission/start', self.start)
        self.create_service(Trigger, '/g17/mission/cancel', self.cancel)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.check_nav)
        self.set_state('idle', 'Localizar en RViz y llamar a /g17/mission/start')

    def set_state(self, state, detail):
        self.state = state
        self.status.publish(String(data=json.dumps(dict(estado=state, detalle=detail))))
        self.get_logger().info(f'{state}: {detail}')

    def on_pose(self, message):
        if message.header.frame_id == 'map':
            self.pose = message.pose.pose

    def check_nav(self):
        if self.state_request is not None and self.state_request.done():
            self.nav_active = (self.state_request.exception() is None
                              and self.state_request.result().current_state.label == 'active')
            self.state_request = None
        if not self.lifecycle.service_is_ready():
            self.nav_active = False
        elif self.state_request is None:
            self.state_request = self.lifecycle.call_async(GetState.Request())

    def start(self, request, response):
        if self.state not in ('idle', 'done', 'failed', 'cancelled'):
            response.message = 'Hay una misión o cancelación en curso'
        elif self.route.get('configured', self.route.get('measured', False)) is not True:
            response.message = 'Primero configurar las poses del mapa'
        elif not self.nav_active or not self.client.server_is_ready() or self.pose is None:
            response.message = 'Esperando Nav2 activo y localización AMCL'
        elif not self.at_initial():
            response.message = 'Colocar y localizar el robot en la pose inicial'
        else:
            self.choice.reset()
            self.send(self.route['door' if self.autonomous else 'predefined'],
                      'to_door' if self.autonomous else 'to_target')
            response.success, response.message = True, 'Misión iniciada'
        return response

    def at_initial(self):
        x, y, yaw = self.route['initial']
        p, q = self.pose.position, self.pose.orientation
        actual = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return (math.hypot(p.x-x, p.y-y) <= self.route.get('start_tolerance', 0.5)
                and abs(math.atan2(math.sin(actual-yaw), math.cos(actual-yaw))) <= 0.6)

    def send(self, pose, state):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        # El destino es fijo en map: sello 0 para usar la transformación más reciente.
        goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, pose[:2])
        goal.pose.pose.orientation.z = math.sin(pose[2]/2)
        goal.pose.pose.orientation.w = math.cos(pose[2]/2)
        self.deadline = time.monotonic() + self.nav_timeout
        self.set_state(state, f'Objetivo: {pose}')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        if future.exception() is not None:
            self.set_state('failed', str(future.exception()))
            return
        self.goal = future.result()
        if not self.goal.accepted:
            self.goal = None
            self.set_state(self.stop_result if self.state == 'cancelling' else 'failed', 'Objetivo rechazado')
            return
        self.goal.get_result_async().add_done_callback(self.on_result)
        if self.state == 'cancelling':
            self.goal.cancel_goal_async()

    def on_result(self, future):
        self.goal = None
        if self.state == 'cancelling':
            self.set_state(self.stop_result, 'Objetivo detenido por Nav2')
        elif future.exception() is not None or future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.set_state('failed', 'Nav2 no ha alcanzado el objetivo')
        elif self.state == 'to_door':
            self.after = self.get_clock().now().nanoseconds/1e9
            self.deadline = time.monotonic() + self.marker_timeout
            self.choice.reset()
            self.set_state('waiting_marker', 'Esperando un ArUco de destino')
        else:
            self.set_state('done', 'Destino alcanzado')

    def on_marker(self, message):
        if self.state != 'waiting_marker':
            return
        try:
            marker = self.choice.update(json.loads(message.data),
                self.get_clock().now().nanoseconds/1e9, self.after)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.choice.reset()
            return
        if marker is not None:
            self.send(self.route['targets'][marker], 'to_target')

    def stop(self, result):
        self.stop_result = result
        if self.state in ('to_door', 'to_target'):
            self.set_state('cancelling', 'Esperando la parada del objetivo')
            if self.goal is not None:
                self.goal.cancel_goal_async()
        else:
            self.set_state(result, 'Misión detenida')

    def cancel(self, request, response):
        if self.state != 'cancelling':
            self.stop('cancelled')
        response.success, response.message = True, 'Parada solicitada'
        return response

    def tick(self):
        if self.state in ('to_door', 'to_target', 'waiting_marker') and time.monotonic() > self.deadline:
            if self.state == 'waiting_marker':
                self.set_state('failed', 'Tiempo agotado esperando leer el ArUco en la puerta')
            else:
                self.stop('failed')


def run(autonomous, args=None):
    rclpy.init(args=args)
    node = Mission(autonomous)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def predefined_main(args=None):
    run(False, args)


def autonomous_main(args=None):
    run(True, args)
