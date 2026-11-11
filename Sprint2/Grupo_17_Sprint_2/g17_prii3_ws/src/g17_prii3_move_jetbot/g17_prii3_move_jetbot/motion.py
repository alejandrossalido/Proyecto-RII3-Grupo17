"""Control físico y recinto de 1 m², comprobables sin ROS.

El recinto se fija una sola vez: x hacia delante, y hacia la izquierda del
robot al arrancar. Los límites incluyen un círculo que contiene el chasis.
Son límites estimados por odometría, no una barrera física ni localización absoluta.
"""

from dataclasses import dataclass
import heapq
import math


# Normaliza un angulo al intervalo de menos pi a pi.
def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Config:
    robot_radius: float = 0.18
    safety_margin: float = 0.03
    max_linear: float = 0.12
    min_linear: float = 0.06
    max_angular: float = 0.65
    min_angular: float = 0.18
    linear_accel: float = 0.30
    angular_accel: float = 1.2
    reaction_time: float = 0.30
    goal_tolerance: float = 0.015
    cross_track_limit: float = 0.035
    linear_scale: float = 1.0
    sensor_timeout: float = 0.25
    scan_timeout: float = 0.35
    waypoint_timeout: float = 35.0

    # Valida que velocidades, dimensiones y margenes sean admisibles.
    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0
               for value in self.__dict__.values()):
            raise ValueError('Los parámetros deben ser finitos y positivos')
        if self.min_linear > self.max_linear or self.min_angular > self.max_angular:
            raise ValueError('La velocidad mínima supera a la máxima')
        if self.robot_radius + self.safety_margin >= 0.265:
            raise ValueError('Chasis/margen demasiado grande para esta ruta de 1 m²')
        if self.max_linear > 0.20 or self.max_angular > 1.0:
            raise ValueError('Velocidad excesiva para esta superficie')


# Misma forma del simulador, ancho de 44 cm y alto de 30,46 cm.
SCALE = 0.44 / 2.6
WAYPOINTS = tuple((0.28 + x * SCALE, 0.72 + y * SCALE, draw, stroke)
                  for x, y, draw, stroke in (
                      (0, -1.8, True, 0), (1.2, -1.8, False, None),
                      (1.2, 0, False, None), (2.6, 0, True, 1),
                      (1.7, -1.8, True, 1)))
START = (0.28, 0.72)


class Workspace:
    """Cuadrado fijo de lado 1 m; reiniciar el dibujo no lo desplaza."""

    # Prepara los limites del recinto y su referencia inicial.
    def __init__(self, config):
        self.config = config
        self.anchor = None

    # Transforma la odometria a coordenadas del recinto fijado al arrancar.
    def pose(self, odom_pose):
        if self.anchor is None:
            self.anchor = odom_pose
        ax, ay, heading = self.anchor
        dx = (odom_pose[0] - ax) * self.config.linear_scale
        dy = (odom_pose[1] - ay) * self.config.linear_scale
        return (START[0] + dx * math.cos(heading) + dy * math.sin(heading),
                START[1] - dx * math.sin(heading) + dy * math.cos(heading),
                wrap(odom_pose[2] - heading))

    # Comprueba que el centro deja espacio para el chasis y su margen.
    def contains(self, point, reserve=0.0):
        pad = self.config.robot_radius + self.config.safety_margin + reserve
        return all(math.isfinite(v) and pad <= v <= 1.0 - pad for v in point[:2])

    # Comprueba que el robot dispone de espacio para frenar dentro del recinto.
    def permits(self, pose, velocity, measured_speed=0.0):
        # Circulo conservador alrededor del centro: cubre giro, reaccion y
        # frenado en cualquier direccion, tambien si la orden nueva es menor.
        speed = max(abs(velocity), abs(measured_speed))
        stopping = speed * self.config.reaction_time + speed**2 / (
            2 * self.config.linear_accel)
        return self.contains(pose, stopping)


class SegmentFollower:
    """Gira en el sitio y sigue rectas, con aceleración limitada."""

    # Prepara el seguidor de segmentos con la configuracion de movimiento.
    def __init__(self, config):
        self.config = config
        self.reset()

    # Borra el objetivo y reinicia la alineacion y las velocidades previas.
    def reset(self):
        self.target = None
        self.aligning = True
        self.last_v = self.last_w = 0.0

    # Pone a cero las velocidades internas del seguidor.
    def stop(self):
        self.last_v = self.last_w = 0.0

    # Calcula avance y giro para seguir el segmento y detectar desviaciones.
    def command(self, pose, start, target, dt, speed_limit=None):
        """Devuelve (v, w, alcanzado, fallo); nunca corrige marcha atrás."""
        c = self.config
        if self.target != target:
            self.reset()
            self.target = target
        x, y, yaw = pose
        length = math.hypot(target[0] - start[0], target[1] - start[1])
        if length < c.goal_tolerance:
            self.stop()
            return 0.0, 0.0, True, None
        course = math.atan2(target[1] - start[1], target[0] - start[0])
        along = (x-start[0])*math.cos(course) + (y-start[1])*math.sin(course)
        cross = -(x-start[0])*math.sin(course) + (y-start[1])*math.cos(course)
        remaining = length - along
        if abs(cross) > c.cross_track_limit:
            self.stop()
            return 0.0, 0.0, False, 'Desviación lateral excesiva'
        if remaining < -c.goal_tolerance:
            self.stop()
            return 0.0, 0.0, False, 'Objetivo sobrepasado: revisar frenado/odometría'
        if remaining <= c.goal_tolerance:
            self.stop()
            return 0.0, 0.0, True, None

        error = wrap(course - math.atan2(cross, 0.12) - yaw)
        if abs(error) > math.radians(12):
            self.aligning = True
        if self.aligning and abs(error) <= math.radians(3):
            self.aligning = False
        w = max(-c.max_angular, min(c.max_angular, 2.0 * error))
        if self.aligning:
            w = math.copysign(max(c.min_angular, abs(w)), error)
            v = 0.0
        else:
            # Reduce velocidad por distancia restante y error de rumbo.
            maximum = c.max_linear if speed_limit is None else speed_limit
            v = min(maximum, max(c.min_linear, 0.9 * remaining))
            v *= max(0.0, math.cos(error))
        dt = min(0.1, max(0.001, dt))
        v = max(self.last_v-c.linear_accel*dt,
                min(self.last_v+c.linear_accel*dt, v))
        # Nunca avanzar mientras se esta alineando, ni siquiera por la rampa.
        if self.aligning:
            v = 0.0
        w = max(self.last_w-c.angular_accel*dt,
                min(self.last_w+c.angular_accel*dt, w))
        self.last_v, self.last_w = v, w
        return v, w, False, None


# Obtiene la distancia minima valida de un sector LiDAR corregido.
def scan_sector(scan, start, end, yaw_offset=0.0, sensor_x=0.0, sensor_y=0.0):
    """Rangos respecto a base_link, en radianes. None = sector sin datos.

    +inf significa sin retorno, NaN/0 no acreditan vía libre, -inf y lecturas
    positivas por debajo del mínimo se tratan como demasiado cerca.
    """
    minimum = math.inf
    usable = 0
    if not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        return None
    for i, value in enumerate(scan.ranges):
        angle = wrap(scan.angle_min + i*scan.angle_increment + yaw_offset)
        if value == math.inf:
            bearing, distance = angle, math.inf
        elif value == -math.inf or 0 < value < scan.range_min:
            bearing, distance = angle, 0.0
        elif math.isfinite(value) and scan.range_min <= value <= scan.range_max and value > 0:
            x = sensor_x + value*math.cos(angle)
            y = sensor_y + value*math.sin(angle)
            bearing, distance = math.atan2(y, x), math.hypot(x, y)
        else:
            continue
        if start <= bearing <= end:
            usable += 1
            minimum = min(minimum, distance)
    return minimum if usable else None


# Rodeo de obstaculos dentro del mismo recinto.
CELL = 0.025


# Convierte las lecturas LiDAR en posiciones de obstaculos dentro del recinto.
def scan_points(scan, pose, yaw_offset=0.0, sensor_x=0.0, sensor_y=0.0):
    """Convierte retornos LiDAR en puntos del recinto; agrupa a 1 cm."""
    points = set()
    x, y, yaw = pose
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index*scan.angle_increment + yaw_offset
        bx = sensor_x + distance*math.cos(angle)
        by = sensor_y + distance*math.sin(angle)
        px = x + bx*math.cos(yaw) - by*math.sin(yaw)
        py = y + bx*math.sin(yaw) + by*math.cos(yaw)
        points.add((round(px, 2), round(py, 2)))
    return points


# Comprueba que un segmento deja espacio suficiente alrededor de los obstaculos.
def path_clear(start, target, obstacles, clearance):
    """¿Cabe el chasis en toda la recta, incluyendo los extremos?"""
    dx, dy = target[0]-start[0], target[1]-start[1]
    length_squared = dx*dx + dy*dy
    for x, y in obstacles:
        along = 0.0 if length_squared == 0 else max(0.0, min(1.0,
            ((x-start[0])*dx + (y-start[1])*dy)/length_squared))
        if (x-start[0]-along*dx)**2 + (y-start[1]-along*dy)**2 < clearance**2:
            return False
    return True


# Busca con A* un rodeo que respete los obstaculos y el recinto.
def plan_detour(workspace, start, target, obstacles):
    """A*: devuelve puntos de rodeo hasta target, o [] si no cabe.

    Mantiene margen para frenado y error de seguimiento. Si el objetivo está
    ocupado, quien dibuja podrá probar el siguiente vértice del mismo 17.
    """
    start, target = tuple(start[:2]), tuple(target[:2])
    c = workspace.config
    clearance = c.robot_radius + c.safety_margin + c.goal_tolerance + 0.005
    reserve = c.min_linear*c.reaction_time + c.min_linear**2/(2*c.linear_accel)
    reserve += c.goal_tolerance
    if not workspace.contains(target, reserve):
        return []
    if not path_clear(target, target, obstacles, clearance):
        return []
    if path_clear(start, target, obstacles, clearance):
        return [target]

    # Solo se admiten celdas donde quepan el chasis y su margen.
    cells = {(i, j): (i*CELL, j*CELL) for i in range(41) for j in range(41)
             if workspace.contains((i*CELL, j*CELL), reserve)
             and path_clear((i*CELL, j*CELL), (i*CELL, j*CELL), obstacles, clearance)}

    # Encuentra celdas cercanas conectadas mediante segmentos libres.
    def connectors(point, margin):
        nearest = sorted(cells, key=lambda key: math.dist(point, cells[key]))
        return [key for key in nearest if path_clear(point, cells[key], obstacles, margin)][:8]

    # Si aparece un objeto cerca, la primera recta puede salir del margen
    # ampliado, siempre respetando el radio fisico mas el margen de seguridad.
    start_edges = connectors(start, c.robot_radius+c.safety_margin+c.goal_tolerance)
    goal_edges = set(connectors(target, clearance))
    source, destination = (-1, -1), (-2, -2)
    cells.update({source: start, destination: target})
    costs, previous, queue = {source: 0.0}, {}, [(0.0, source)]
    while queue:
        _, current = heapq.heappop(queue)
        if current == destination:
            route = []
            while current != source:
                route.append(cells[current])
                current = previous[current]
            route.reverse()
            # Quitar escalones de cuadricula que se puedan unir con una recta libre.
            simplified, point = [], start
            while route:
                margin = (c.robot_radius+c.safety_margin+c.goal_tolerance
                          if point == start else clearance)
                last = max(i for i, candidate in enumerate(route)
                           if path_clear(point, candidate, obstacles, margin))
                point = route[last]
                simplified.append(point)
                route = route[last+1:]
            return simplified
        if current == source:
            neighbors = start_edges
        else:
            i, j = current
            neighbors = [(i+di, j+dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)
                         if (di or dj) and (i+di, j+dj) in cells
                         and (i+di, j) in cells and (i, j+dj) in cells]
            if current in goal_edges:
                neighbors.append(destination)
        for neighbor in neighbors:
            margin = (c.robot_radius+c.safety_margin+c.goal_tolerance
                      if current == source else clearance)
            if not path_clear(cells[current], cells[neighbor], obstacles, margin):
                continue
            cost = costs[current] + math.dist(cells[current], cells[neighbor])
            if cost < costs.get(neighbor, math.inf):
                costs[neighbor], previous[neighbor] = cost, current
                heapq.heappush(queue, (cost+math.dist(cells[neighbor], target), neighbor))
    return []
