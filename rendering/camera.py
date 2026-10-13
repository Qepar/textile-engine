"""
Camera Class for Textile Engine
Handles 3D camera viewing and navigation.
"""
import numpy as np
from utils.math_utils import norm

class Camera:
    def __init__(self, position=np.array([0, 0, 5]), target=np.array([0, 0, 0]), up=np.array([0, 1, 0])):
        self.position = np.array(position, dtype=np.float64)
        self.target = np.array(target, dtype=np.float64)
        self.up = np.array(up, dtype=np.float64)

    def apply(self):
        from OpenGL.GLU import gluLookAt
        gluLookAt(
            self.position[0], self.position[1], self.position[2],
            self.target[0], self.target[1], self.target[2],
            self.up[0], self.up[1], self.up[2]
        )

    def move_forward(self, distance):
        direction = self.target - self.position
        if norm(direction) > 0:
            direction = direction / norm(direction)
        else:
            direction = np.array([0, 0, -1])
        self.position += direction * distance
        self.target += direction * distance

    def move_right(self, distance):
        direction = self.target - self.position
        if norm(direction) > 0:
            direction = direction / norm(direction)
        else:
            direction = np.array([0, 0, -1])
        right = np.cross(direction, self.up)
        if norm(right) > 0:
            right = right / norm(right)
        else:
            right = np.array([1, 0, 0])
        self.position += right * distance
        self.target += right * distance
