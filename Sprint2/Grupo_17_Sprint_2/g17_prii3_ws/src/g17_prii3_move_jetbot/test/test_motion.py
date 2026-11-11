"""Regresiones de geometría, sensores y seguimiento físico sin ROS."""

import math
from types import SimpleNamespace

import pytest

from g17_prii3_move_jetbot.motion import (
    Config, SegmentFollower, START, WAYPOINTS, Workspace, scan_sector, path_clear, plan_detour, scan_points)


# Crea un barrido LiDAR sencillo para las pruebas de geometria.
def scan(ranges, angle=0.0, increment=0.01):
    return SimpleNamespace(ranges=ranges, angle_min=angle, angle_increment=increment,
                           range_min=0.10, range_max=8.0)


# Comprueba que el chasis y los puntos del 17 caben en el cuadrado.
def test_body_and_waypoints_fit_square():
    ws = Workspace(Config())
    assert all(ws.contains(p) for p in (START, *WAYPOINTS))
    xs, ys = zip(*[p[:2] for p in (START, *WAYPOINTS)])
    assert max(xs)-min(xs) == pytest.approx(0.44)
    assert max(ys)-min(ys) == pytest.approx(1.8*0.44/2.6)
    assert not ws.contains((0.19, 0.5))
    assert not ws.contains((math.nan, 0.5))


# Comprueba que el recinto conserva su referencia y orientacion iniciales.
@pytest.mark.parametrize('heading', [-math.pi, -1.3, 0, 0.9, math.pi])
def test_workspace_is_fixed_to_initial_heading(heading):
    ws = Workspace(Config())
    ws.pose((10, -4, heading))
    pose = ws.pose((10+0.2*math.cos(heading), -4+0.2*math.sin(heading), heading))
    assert pose == pytest.approx((0.48, 0.72, 0))
    anchor = ws.anchor
    ws.pose((11, -3, heading))
    assert ws.anchor == anchor


# Comprueba que el margen de frenado considera la velocidad medida.
def test_braking_reserve_uses_measured_speed():
    ws = Workspace(Config())
    assert ws.contains((0.75, 0.5))
    assert not ws.permits((0.75, 0.5, 0), 0, measured_speed=0.12)
    assert ws.permits((0.50, 0.5, 0), 0.12)


# Comprueba que se rechazan configuraciones de movimiento no validas.
@pytest.mark.parametrize('bad', [dict(max_linear=math.nan), dict(min_linear=0.2),
                                     dict(robot_radius=0.4), dict(max_linear=0.4)])
def test_bad_config_rejected(bad):
    with pytest.raises(ValueError):
        Config(**bad)


# Comprueba el tratamiento de lecturas LiDAR invalidas y su orientacion.
def test_scanner_unknown_is_not_clear():
    assert scan_sector(scan([math.nan, 0.0, math.nan]), -1, 1) is None
    assert scan_sector(scan([math.inf]), -1, 1) == math.inf
    assert scan_sector(scan([-math.inf]), -1, 1) == 0
    assert scan_sector(scan([0.03]), -1, 1) == 0
    assert scan_sector(scan([1.0], angle=math.pi), -0.5, 0.5) is None
    assert scan_sector(scan([1.0], angle=math.pi), -0.5, 0.5, math.pi) == 1.0
    assert scan_sector(scan([0.5]), -0.5, 0.5, sensor_x=0.1) == pytest.approx(0.6)


# Comprueba la alineacion inicial y la deteccion de desvios excesivos.
def test_turn_before_moving_and_fault_on_large_deviation():
    follower = SegmentFollower(Config())
    v, w, reached, fault = follower.command((0.3, 0.5, math.pi/2), (0.3, 0.5), (0.6, 0.5), 0.05)
    assert v == 0 and w < 0 and not reached and fault is None
    assert follower.command((0.3, 0.6, 0), (0.3, 0.5), (0.6, 0.5), 0.05)[3]
    assert follower.command((0.65, 0.5, 0), (0.3, 0.5), (0.6, 0.5), 0.05)[3]


# Comprueba que el rodeo respeta el tamaño del chasis y los limites.
def test_detour_avoids_body_collision_and_stays_inside():
    ws = Workspace(Config())
    obstacles = [(0.48, 0.5), (0.5, 0.48), (0.52, 0.5), (0.5, 0.52)]
    start, target = (0.28, 0.72), (0.72, 0.28)
    route = plan_detour(ws, start, target, obstacles)
    assert route and route[-1] == target
    for point in route:
        assert ws.contains(point)
        assert path_clear(start, point, obstacles, 0.225)
        start = point


# Comprueba que no se propone un rodeo cuando el paso esta cerrado.
def test_blocked_target_or_closed_corridor_returns_no_route():
    ws = Workspace(Config())
    assert plan_detour(ws, (0.28, 0.72), (0.72, 0.28), [(0.72, 0.28)]) == []
    wall = [(0.5, i/100) for i in range(101)]
    assert plan_detour(ws, (0.28, 0.72), (0.72, 0.28), wall) == []


# Comprueba la transformacion de lecturas LiDAR a posiciones del recinto.
def test_scan_points_use_robot_pose_and_mounting():
    points = scan_points(scan([0.5]), (0.3, 0.4, math.pi/2), sensor_x=0.1)
    assert points == {(0.3, 1.0)}


# Comprueba el recorrido completo con retardo y zona muerta de los motores.
@pytest.mark.parametrize('lag,deadband', [(0.0, 0.0), (0.12, 0.04)])
def test_complete_number_with_actuator_lag(lag, deadband):
    config = Config()
    ws, follower = Workspace(config), SegmentFollower(config)
    pose = [*START, 0.0]
    leg_start, total_steps, v_real, w_real = START, 0, 0.0, 0.0
    dt = 0.05
    for waypoint in WAYPOINTS:
        target = waypoint[:2]
        for _ in range(900):
            total_steps += 1
            v, w, reached, fault = follower.command(pose, leg_start, target, dt)
            assert fault is None
            assert ws.permits(pose, v, v_real)
            assert abs(v) <= config.max_linear and abs(w) <= config.max_angular
            alpha = 1.0 if not lag else dt/(lag+dt)
            v_real += alpha*(v-v_real)
            w_real += alpha*(w-w_real)
            actual_v = 0 if abs(v_real) < deadband else v_real
            pose[0] += actual_v*math.cos(pose[2]+w_real*dt/2)*dt
            pose[1] += actual_v*math.sin(pose[2]+w_real*dt/2)*dt
            pose[2] += w_real*dt
            assert ws.contains(pose)
            if reached:
                # Tiempo de asentamiento del chasis, como en el nodo fisico.
                for _ in range(5):
                    v_real += alpha*(0-v_real)
                    w_real += alpha*(0-w_real)
                    pose[0] += v_real*math.cos(pose[2])*dt
                    pose[1] += v_real*math.sin(pose[2])*dt
                    pose[2] += w_real*dt
                break
        else:
            pytest.fail('No termina el tramo')
        leg_start = target
    assert total_steps*dt < 90
