"""Poses en metros/radianes de map y selección inequívoca de un ArUco."""
import math
import yaml


def load_route(path):
    with open(path, encoding='utf-8') as stream:
        route = yaml.safe_load(stream)
    for name in ('initial', 'door', 'predefined'):
        validate_pose(route[name])
    targets = route['targets']
    if len(targets) != 3 or any(type(key) is not int or not 0 <= key < 100 for key in targets):
        raise ValueError('Se necesitan exactamente tres IDs enteros distintos entre 0 y 99')
    for pose in targets.values():
        validate_pose(pose)
    if len({tuple(pose) for pose in targets.values()}) != 3:
        raise ValueError('Los tres destinos deben ser distintos')
    return route


def validate_pose(pose):
    if len(pose) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in pose):
        raise ValueError('Cada pose debe tener [x, y, yaw] finitos')


class MarkerChoice:
    """Exige varias imágenes consecutivas recientes con un solo ID conocido."""
    def __init__(self, targets, frames=3, max_age=0.8):
        self.targets, self.frames, self.max_age = targets, frames, max_age
        self.reset()

    def reset(self):
        self.candidate, self.count, self.last_stamp = None, 0, -math.inf

    def update(self, data, now, after):
        stamp = float(data['frame_stamp'])
        ids = {int(marker['id']) for marker in data['aruco']} & self.targets.keys()
        valid = (data['estado'] == 'ok' and math.isfinite(stamp)
                 # Gazebo puede entregar la imagen antes del siguiente /clock.
                 and -0.2 <= now - stamp <= self.max_age and stamp > after
                 and stamp > self.last_stamp and len(ids) == 1)
        if not valid:
            self.reset()
            return None
        marker = ids.pop()
        consecutive = marker == self.candidate and stamp - self.last_stamp <= self.max_age
        self.count = self.count + 1 if consecutive else 1
        self.candidate, self.last_stamp = marker, stamp
        return marker if self.count >= self.frames else None
