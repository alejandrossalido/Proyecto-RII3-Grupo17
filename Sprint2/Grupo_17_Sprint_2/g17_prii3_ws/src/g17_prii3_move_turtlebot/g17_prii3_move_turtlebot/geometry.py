"""Geometría pequeña y comprobable sin iniciar ROS."""

import math


# Normaliza un angulo al intervalo de menos pi a pi.
def wrap_angle(angle):
    """Devuelve un ángulo en [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Transforma un punto del dibujo al marco de odometria.
def local_to_world(origin, forward, left):
    """Transforma coordenadas del dibujo al marco de odometría."""
    # Rota el punto segun la orientacion inicial y lo desplaza al origen.
    x, y, yaw = origin
    return (
        x + forward * math.cos(yaw) - left * math.sin(yaw),
        y + forward * math.sin(yaw) + left * math.cos(yaw),
    )


# Busca la distancia valida mas cercana dentro de un sector LiDAR.
def sector_min(scan, start_degrees, end_degrees):
    """Distancia mínima válida en un sector LiDAR; infinito si está vacío."""
    # Descarta lecturas invalidas y conserva la mas cercana del sector.
    minimum = math.inf
    for index, distance in enumerate(scan.ranges):
        angle = wrap_angle(scan.angle_min + index * scan.angle_increment)
        degrees = math.degrees(angle)
        if start_degrees <= degrees <= end_degrees:
            if (math.isfinite(distance) and
                    scan.range_min <= distance <= scan.range_max):
                minimum = min(minimum, distance)
    return minimum


# Obtiene la distancia minima del sector frontal del LiDAR.
def front_min(scan):
    """Combina los dos sectores que rodean los 0 grados del LiDAR."""
    # El sector frontal se centra en 0°.
    return sector_min(scan, -25, 25)


# Devuelve la distancia y el angulo del obstaculo mas cercano del sector.
def nearest_in_sector(scan, start_degrees, end_degrees):
    """Devuelve (distancia, ángulo) del obstáculo más cercano del sector."""
    nearest = (math.inf, 0.0)
    for index, distance in enumerate(scan.ranges):
        angle = wrap_angle(scan.angle_min + index * scan.angle_increment)
        degrees = math.degrees(angle)
        if start_degrees <= degrees <= end_degrees:
            if (math.isfinite(distance) and
                    scan.range_min <= distance <= scan.range_max):
                if distance < nearest[0]:
                    nearest = (distance, angle)
    return nearest
