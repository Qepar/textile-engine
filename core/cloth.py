"""
Cloth Class for Textile Engine
Represents a cloth mesh made of particles and constraints.
"""
import numpy as np
from .particle import Particle
from .constraints import DistanceConstraint, BendConstraint, ShearConstraint

class Cloth:
    """
    A cloth simulation object composed of particles and constraints.
    """
    def __init__(self, width=2.0, height=2.0, num_width=10, num_height=10, mass=0.1, stiffness=10.0, use_anisotropic=False, use_wet_simulation=False):
        self.width = width
        self.height = height
        self.num_width = num_width
        self.num_height = num_height
        self.particles = []
        self.constraints = []
        self.use_anisotropic = use_anisotropic
        self.use_wet_simulation = use_wet_simulation
        self._init_particles(mass)
        self._init_constraints(mass, stiffness)

    def _init_particles(self, mass):
        dx = self.width / (self.num_width - 1) if self.num_width > 1 else self.width
        dy = self.height / (self.num_height - 1) if self.num_height > 1 else self.height
        for i in range(self.num_height):
            for j in range(self.num_width):
                x = -self.width / 2 + j * dx
                y = -self.height / 2 + i * dy
                fixed = (i == 0 and (j == 0 or j == self.num_width - 1))
                self.particles.append(Particle([x, y, 0], mass=mass, fixed=fixed))

    def _init_constraints(self, mass, stiffness):
        for i in range(self.num_height):
            for j in range(self.num_width):
                idx = i * self.num_width + j
                if j < self.num_width - 1:
                    rest_length = self.width / (self.num_width - 1)
                    if self.use_anisotropic:
                        self.constraints.append(AnisotropicDistanceConstraint(idx, idx + 1, rest_length, stiffness_x=stiffness * 1.2, stiffness_y=stiffness * 0.8))
                    elif self.use_wet_simulation:
                        self.constraints.append(WetDistanceConstraint(idx, idx + 1, rest_length, stiffness))
                    else:
                        self.constraints.append(DistanceConstraint(idx, idx + 1, rest_length, stiffness))
                if i < self.num_height - 1:
                    rest_length = self.height / (self.num_height - 1)
                    if self.use_anisotropic:
                        self.constraints.append(AnisotropicDistanceConstraint(idx, idx + self.num_width, rest_length, stiffness_x=stiffness * 1.2, stiffness_y=stiffness * 0.8, direction=np.array([0, 1, 0])))
                    elif self.use_wet_simulation:
                        self.constraints.append(WetDistanceConstraint(idx, idx + self.num_width, rest_length, stiffness))
                    else:
                        self.constraints.append(DistanceConstraint(idx, idx + self.num_width, rest_length, stiffness))
        for i in range(self.num_height - 1):
            for j in range(self.num_width - 1):
                idx = i * self.num_width + j
                rest_length = np.sqrt((self.width / (self.num_width - 1)) ** 2 + (self.height / (self.num_height - 1)) ** 2)
                self.constraints.append(DistanceConstraint(idx, idx + self.num_width + 1, rest_length, stiffness * 0.7))
                self.constraints.append(DistanceConstraint(idx + 1, idx + self.num_width, rest_length, stiffness * 0.7))
                self.constraints.append(BendConstraint(idx, idx + 1, idx + self.num_width + 1, stiffness * 0.3))
                self.constraints.append(BendConstraint(idx + 1, idx, idx + self.num_width, stiffness * 0.3))

    def apply_external_force(self, force):
        for p in self.particles:
            if not p.fixed:
                p.force += force

    def apply_wind(self, wind_force):
        for p in self.particles:
            if not p.fixed:
                height_factor = 1.0 + 0.5 * (p.position[2] / (self.height / 2))
                p.force += wind_force * height_factor

    def set_dampness(self, dampness):
        for p in self.particles:
            p.dampness = dampness

    def set_temperature(self, temperature):
        for p in self.particles:
            p.temperature = temperature

    def reset(self):
        dx = self.width / (self.num_width - 1) if self.num_width > 1 else self.width
        dy = self.height / (self.num_height - 1) if self.num_height > 1 else self.height
        for i, p in enumerate(self.particles):
            j = i % self.num_width
            k = i // self.num_width
            p.position = np.array([-self.width / 2 + j * dx, -self.height / 2 + k * dy, 0])
            p.velocity = np.zeros(3)
            p.force = np.zeros(3)
            p.prev_position = p.position.copy()
