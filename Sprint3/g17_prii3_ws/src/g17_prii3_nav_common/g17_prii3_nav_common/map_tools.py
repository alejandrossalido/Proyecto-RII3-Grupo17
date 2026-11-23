"""Lee el mapa ROS y obtiene las paredes para construirlas en Gazebo."""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class Map:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.config = yaml.safe_load(self.path.read_text())
        self.image = cv2.imread(str(self.path.parent / self.config['image']), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError('No se encuentra la imagen asociada al mapa')
        self.resolution = float(self.config['resolution'])
        self.ox, self.oy, self.yaw = map(float, self.config['origin'])
        probability = self.image/255. if self.config.get('negate', 0) else 1-self.image/255.
        self.occupied = probability > self.config['occupied_thresh']

    def world(self, col, row):
        x, y = col*self.resolution, (self.image.shape[0]-row)*self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.ox+c*x-s*y, self.oy+s*x+c*y

    def rectangles(self):
        """Agrupa píxeles de pared en rectángulos: menos geometría para Gazebo."""
        active, finished = {}, []
        for row, line in enumerate(self.occupied):
            edges = np.flatnonzero(np.diff(np.r_[False, line, False])).reshape(-1, 2)
            runs = {tuple(edge) for edge in edges}
            for run in active.keys()-runs:
                first, last = active.pop(run)
                finished.append((*run, first, last))
            for run in runs:
                active[run] = (active.get(run, (row, row))[0], row+1)
        return finished + [(*run, first, last) for run, (first, last) in active.items()]
