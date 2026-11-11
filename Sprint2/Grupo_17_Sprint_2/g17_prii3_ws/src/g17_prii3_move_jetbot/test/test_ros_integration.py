"""ROS real con chasis ficticio; dominio de pruebas aislado, sin hardware."""

import math
import os
import time

import pytest
# Aislamiento obligatorio incluso si pytest se invoca desde el entorno operativo.
os.environ['ROS_DOMAIN_ID'] = '231'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from g17_prii3_move_jetbot.controller import JetbotDrawingController
from g17_prii3_move_jetbot.motion import START, WAYPOINTS


# Prepara ROS con un dominio y topics aislados para las pruebas.
@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=['--ros-args',
        '-p', 'cmd_vel_topic:=/g17_test_only/cmd_vel',
        '-p', 'odom_topic:=/g17_test_only/odom',
        '-p', 'scan_topic:=/g17_test_only/scan',
        '-r', '/continue_drawing:=/g17_test_only/continue_drawing',
        '-r', '/stop_drawing:=/g17_test_only/stop_drawing',
        '-r', '/restart_drawing:=/g17_test_only/restart_drawing',
        '-r', '/g17/drawing:=/g17_test_only/drawing'])
    yield
    rclpy.shutdown()


# Prepara un controlador y un chasis ficticio conectados mediante ROS.
@pytest.fixture
def rig(request):
    controller = JetbotDrawingController(getattr(request, 'param', 'draw_number'))
    controller.timer.cancel()
    plant = Node('g17_test_chassis')
    odom_pub = plant.create_publisher(Odometry, '/g17_test_only/odom', 10)
    scan_pub = plant.create_publisher(LaserScan, '/g17_test_only/scan', 10)
    commands = []
    plant.create_subscription(Twist, '/g17_test_only/cmd_vel', commands.append, 10)

    # Procesa los mensajes pendientes del controlador y del chasis ficticio.
    def spin():
        for _ in range(3):
            rclpy.spin_once(controller, timeout_sec=0.001)
            rclpy.spin_once(plant, timeout_sec=0.001)

    # Publica odometria y lecturas LiDAR ficticias para alimentar el controlador.
    def feed(pose=(0., 0., 0.), speed=0., ranges=None):
        odom = Odometry()
        odom.header.stamp = plant.get_clock().now().to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose[:2]
        odom.pose.pose.orientation.z = math.sin(pose[2]/2)
        odom.pose.pose.orientation.w = math.cos(pose[2]/2)
        odom.twist.twist.linear.x = float(speed)
        scan = LaserScan()
        scan.header.stamp = odom.header.stamp
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, math.pi/180
        scan.range_min, scan.range_max = 0.10, 8.0
        scan.ranges = ranges if ranges is not None else [math.inf]*361
        odom_pub.publish(odom)
        scan_pub.publish(scan)
        spin()
        return odom

    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        feed()
        controller.graph_at = 0
        controller.check_graph()
        if controller.ready_reason() is None:
            break
    assert controller.ready_reason() is None
    yield controller, plant, feed, spin, commands
    controller.paused = True
    controller.destroy_node()
    plant.destroy_node()


# Solicita continuar el dibujo y comprueba que la orden se acepta.
def resume(c):
    response = c.on_continue(Trigger.Request(), Trigger.Response())
    assert response.success, response.message


# Comprueba el arranque pausado y la reanudacion mediante un servicio ROS.
def test_start_paused_and_service_over_dds(rig):
    c, plant, feed, spin, commands = rig
    c.control()
    spin()
    assert c.paused and all(m.linear.x == m.angular.z == 0 for m in commands)
    client = plant.create_client(Trigger, '/continue_drawing')
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic()+2
    while not future.done() and time.monotonic() < deadline:
        feed()
    assert future.result().success and not c.paused
    plant.destroy_client(client)


# Comprueba que perder la odometria provoca una parada enclavada.
def test_loss_of_odom_latches_stop(rig):
    c, _, _, spin, commands = rig
    resume(c)
    c.control()
    c.odom_at -= 1
    c.control()
    spin()
    assert c.fault and c.paused
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    assert not c.on_continue(Trigger.Request(), Trigger.Response()).success


# Comprueba que perder el LiDAR provoca una parada enclavada.
@pytest.mark.parametrize('rig', ['collision_avoidance'], indirect=True)
def test_loss_of_scan_latches_stop(rig):
    c, _, _, _, _ = rig
    resume(c)
    c.control()
    c.scan_at -= 2
    c.control()
    assert c.fault and c.paused


# Comprueba la parada cuando aparecen publicadores de velocidad duplicados.
def test_duplicate_velocity_publisher_stops(rig):
    c, plant, feed, _, _ = rig
    other = plant.create_publisher(Twist, '/g17_test_only/cmd_vel', 10)
    deadline = time.monotonic()+2
    while c.count_publishers('/g17_test_only/cmd_vel') < 2 and time.monotonic() < deadline:
        feed()
    c.graph_at = 0
    c.control()
    assert c.fault and not c.graph_ok
    plant.destroy_publisher(other)


# Comprueba fallos de odometria y las restricciones para reiniciar.
def test_odom_jump_invalid_quaternion_and_restart_bounds(rig):
    c, _, feed, _, _ = rig
    anchor = c.workspace.anchor
    msg = feed()
    msg.header.stamp = c.get_clock().now().to_msg()
    msg.pose.pose.position.x = 2.0
    c.on_odom(msg)
    assert c.fault
    assert c.workspace.anchor == anchor
    # Reiniciar no abre un cuadrado nuevo alrededor de la posicion actual.
    c.pose = (0.5, 0.5, 0.0)
    assert not c.on_restart(Trigger.Request(), Trigger.Response()).success
    assert c.workspace.anchor == anchor
    c.pose = (*START, 0.0)
    c.on_restart(Trigger.Request(), Trigger.Response())
    msg.pose.pose.orientation.w = msg.pose.pose.orientation.z = 0.0
    c.on_odom(msg)
    assert 'cuaternión' in c.fault


# Comprueba la parada ante lecturas invalidas u obstaculos demasiado proximos.
@pytest.mark.parametrize('rig', ['collision_avoidance'], indirect=True)
def test_all_invalid_scan_and_close_obstacle_command_zero(rig):
    c, _, feed, spin, commands = rig
    resume(c)
    feed(ranges=[math.nan]*361)
    c.control()
    spin()
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    feed(ranges=[0.15]*361)
    assert c.on_restart(Trigger.Request(), Trigger.Response()).success
    resume(c)
    c.control()
    spin()
    assert commands[-1].linear.x == commands[-1].angular.z == 0


# Comprueba que el modo de parada espera y continua al retirar el obstaculo.
@pytest.mark.parametrize('rig', ['collision_avoidance'], indirect=True)
def test_collision_waits_and_resumes(rig):
    c, _, feed, spin, commands = rig
    resume(c)
    feed(ranges=[0.25]*361)
    c.control()
    spin()
    assert c.blocked
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    feed()
    c.control()
    assert not c.blocked and c.fault is None


# Comprueba que se conserva el progreso cuando no cabe un rodeo.
@pytest.mark.parametrize('rig', ['obstacle_avoidance'], indirect=True)
def test_obstacle_without_room_waits_and_keeps_progress(rig):
    c, _, feed, spin, commands = rig
    resume(c)
    anchor, waypoint = c.workspace.anchor, c.waypoint
    # Anillo de obstaculos: no existe salida, aunque el chasis aun no lo toca.
    feed(ranges=[0.25]*361)
    c.control()
    spin()
    assert c.blocked and not c.detour
    assert c.waypoint == waypoint and c.workspace.anchor == anchor
    assert commands[-1].linear.x == commands[-1].angular.z == 0


# Comprueba la parada enclavada al alcanzar el margen del recinto.
def test_workspace_limit_latches_stop(rig):
    c, _, _, _, _ = rig
    resume(c)
    c.pose = (0.9, 0.5, 0.)
    c.control()
    assert c.fault and c.paused


# Comprueba el recorrido completo mediante mensajes ROS y un chasis ficticio.
@pytest.mark.parametrize('rig', ['draw_number', 'collision_avoidance', 'obstacle_avoidance'], indirect=True)
def test_complete_route_via_ros_topics(rig):
    c, _, feed, spin, commands = rig
    resume(c)
    pose = [0., 0., 0.]
    for _ in range(2400):
        c.last_tick = time.monotonic()-0.05
        c.control()
        spin()
        cmd = commands[-1]
        pose[0] += cmd.linear.x*math.cos(pose[2]+cmd.angular.z*0.025)*0.05
        pose[1] += cmd.linear.x*math.sin(pose[2]+cmd.angular.z*0.025)*0.05
        pose[2] += cmd.angular.z*0.05
        ranges = None
        if c.require_scan:
            # Ray casting contra paredes del propio cuadrado, no solo cielo libre.
            px, py = pose[0]+START[0], pose[1]+START[1]
            ranges = []
            for i in range(361):
                angle = pose[2]-math.pi+i*math.pi/180
                dx, dy = math.cos(angle), math.sin(angle)
                tx = (1-px)/dx if dx > 1e-8 else (-px/dx if dx < -1e-8 else math.inf)
                ty = (1-py)/dy if dy > 1e-8 else (-py/dy if dy < -1e-8 else math.inf)
                ranges.append(min(tx, ty))
        feed(pose, cmd.linear.x, ranges=ranges)
        assert c.fault is None, c.fault
        assert c.workspace.contains(c.pose)
        if c.waypoint == len(WAYPOINTS):
            break
    assert c.waypoint == len(WAYPOINTS)
    c.control()
    spin()
    assert commands[-1].linear.x == commands[-1].angular.z == 0


# Comprueba que mensajes antiguos o repetidos no cuentan como odometria reciente.
def test_stale_and_repeated_stamps_do_not_refresh_odom(rig):
    c, _, feed, _, _ = rig
    msg = feed()
    before = c.odom_at
    c.on_odom(msg)
    assert c.odom_at == before
    msg.header.stamp.sec -= 2
    c.on_odom(msg)
    assert c.odom_at == before


# Comprueba que el diagnostico recibe sensores sin publicar velocidades.
def test_passive_diagnostic_sees_sensors_without_publishing(rig):
    from types import SimpleNamespace
    from g17_prii3_move_jetbot.diagnostics import Diagnostic, json_safe
    import json
    c, _, feed, _, _ = rig
    diagnostic = Diagnostic(SimpleNamespace(odom_topic='/g17_test_only/odom',
                            scan_topic='/g17_test_only/scan', cmd_vel_topic='/g17_test_only/cmd_vel'))
    try:
        before = c.count_publishers('/g17_test_only/cmd_vel')
        for _ in range(10):
            feed()
            rclpy.spin_once(diagnostic, timeout_sec=0.01)
            rclpy.spin_once(diagnostic, timeout_sec=0.01)
        report = diagnostic.report()
        assert report['passive'] and report['sensors']['odom']['messages'] > 1
        assert report['sensors']['scan']['usable_rays'] == 361
        assert c.count_publishers('/g17_test_only/cmd_vel') == before == 1
        json.dumps(json_safe(report), allow_nan=False)
    finally:
        diagnostic.destroy_node()


# Genera distancias LiDAR ficticias para una escena con obstaculos circulares.
def scene_scan(raw_pose, circles):
    """LiDAR ficticio: intersecciones reales de rayos con círculos."""
    x, y = raw_pose[0]+START[0], raw_pose[1]+START[1]
    ranges = []
    for index in range(361):
        angle = raw_pose[2]-math.pi+index*math.pi/180
        dx, dy = math.cos(angle), math.sin(angle)
        nearest = math.inf
        for cx, cy, radius in circles:
            rx, ry = cx-x, cy-y
            along = rx*dx+ry*dy
            discriminant = radius**2-(rx*rx+ry*ry-along*along)
            if discriminant >= 0 and along > 0:
                nearest = min(nearest, along-math.sqrt(discriminant))
        ranges.append(nearest)
    return ranges


# Comprueba que el modo simple funciona sin LiDAR ni logica de esquiva.
def test_plain_mode_needs_no_lidar_and_has_no_avoidance(rig):
    c, _, feed, spin, commands = rig
    assert not c.require_scan and not hasattr(c, 'scan_sub')
    resume(c)
    feed(ranges=[math.nan]*361)
    c.control()
    spin()
    assert not c.blocked and not c.detour and c.fault is None
    assert commands[-1].angular.z != 0  # Comienza el giro inicial del 17.


# Comprueba que el rodeo conserva el progreso y termina el mismo 17.
@pytest.mark.parametrize('rig,scenario', [
    ('collision_avoidance', 'wait'),
    ('obstacle_avoidance', 'side'),
    ('obstacle_avoidance', 'vertex')], indirect=['rig'])
def test_obstacle_during_drawing_finishes_same_number(rig, scenario):
    c, _, feed, spin, commands = rig
    resume(c)
    pose = [0., 0., 0.]
    anchor = c.workspace.anchor
    previous_waypoint = 0
    appeared = False
    waited = 0
    planned = skipped_vertex = False
    original_stroke = None
    for step in range(4000):
        if not appeared and ((c.mode == 'collision_avoidance' and c.waypoint == 1)
                             or (c.mode == 'obstacle_avoidance' and pose[1] < -0.004)):
            appeared = True
            original_stroke = list(c.strokes[0])
        if c.mode == 'collision_avoidance':
            circles = [(0.38, 0.20, 0.02)] if appeared and waited < 25 else []
        else:
            obstacle = (0.28, WAYPOINTS[0][1], 0.02) if scenario == 'vertex' else (0.07, 0.565, 0.02)
            circles = [obstacle] if appeared else []
        feed(pose, commands[-1].linear.x if commands else 0., scene_scan(pose, circles))
        c.last_tick = time.monotonic()-0.05
        c.control()
        spin()
        cmd = commands[-1]
        if c.blocked and c.mode == 'collision_avoidance':
            waited += 1
            assert cmd.linear.x == cmd.angular.z == 0
            assert c.waypoint == 1
        planned = planned or bool(c.detour)
        skipped_vertex = skipped_vertex or (bool(c.detour) and c.detour_goal > c.waypoint)
        pose[0] += cmd.linear.x*math.cos(pose[2]+cmd.angular.z*0.025)*0.05
        pose[1] += cmd.linear.x*math.sin(pose[2]+cmd.angular.z*0.025)*0.05
        pose[2] += cmd.angular.z*0.05
        assert c.fault is None, c.fault
        assert c.workspace.anchor == anchor
        assert c.waypoint >= previous_waypoint
        assert c.workspace.contains((pose[0]+START[0], pose[1]+START[1]))
        for cx, cy, radius in circles:
            assert math.hypot(pose[0]+START[0]-cx, pose[1]+START[1]-cy) >= c.config.robot_radius+radius
        previous_waypoint = c.waypoint
        if c.waypoint == len(WAYPOINTS):
            break
    assert c.waypoint == len(WAYPOINTS), (c.pose, c.detour, c.blocked)
    assert appeared and original_stroke
    assert c.strokes[0][:len(original_stroke)] == original_stroke
    if c.mode == 'collision_avoidance':
        assert waited == 25 and not planned
    else:
        assert planned
        if scenario == 'vertex':
            assert skipped_vertex
    c.control()
    spin()
    assert commands[-1].linear.x == commands[-1].angular.z == 0


# Comprueba que no se intenta un rodeo con sectores laterales sin datos.
@pytest.mark.parametrize('rig', ['obstacle_avoidance'], indirect=True)
def test_avoidance_cannot_plan_with_blind_sides(rig):
    c, _, feed, spin, commands = rig
    ranges = [math.nan]*361
    ranges[150:211] = [math.inf]*61
    feed(ranges=ranges)
    response = c.on_continue(Trigger.Request(), Trigger.Response())
    assert not response.success and 'ambos lados' in response.message
    c.control()
    spin()
    assert commands[-1].linear.x == commands[-1].angular.z == 0
