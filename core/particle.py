import numpy as np

class Particle:
    def __init__(self, position, velocity=np.zeros(3), mass=1.0, fixed=False, radius=0.05):
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.force = np.zeros(3, dtype=np.float64)
        self.mass = mass
        self.inv_mass = 1.0 / mass if mass > 0 else 0.0
        self.fixed = fixed
        self.radius = radius
        self.prev_position = self.position.copy()
        self.collision_normal = None
        self.collision_depth = 0.0
