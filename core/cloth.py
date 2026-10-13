import numpy as np
from .particle import Particle
from .constraints import DistanceConstraint, BendConstraint

class Cloth:
    def __init__(self, width=2.0, height=2.0, num_width=10, num_height=10, mass=0.1, stiffness=10.0):
        self.width = width
        self.height = height
        self.num_width = num_width
        self.num_height = num_height
        self.particles = []
        self.constraints = []
        self._init_particles()
        self._init_constraints(mass, stiffness)

    def _init_particles(self):
        dx = self.width / (self.num_width - 1) if self.num_width > 1 else self.width
        dy = self.height / (self.num_height - 1) if self.num_height > 1 else self.height
        for i in range(self.num_height):
            for j in range(self.num_width):
                x = -self.width/2 + j * dx
                y = -self.height/2 + i * dy
                fixed = (i == 0 and (j == 0 or j == self.num_width-1))
                self.particles.append(Particle([x, y, 0], mass=0.1, fixed=fixed))

    def _init_constraints(self, mass, stiffness):
        for i in range(self.num_height):
            for j in range(self.num_width):
                idx = i * self.num_width + j
                if j < self.num_width - 1:
                    rest_length = self.width / (self.num_width - 1)
                    self.constraints.append(DistanceConstraint(idx, idx+1, rest_length, stiffness))
                if i < self.num_height - 1:
                    rest_length = self.height / (self.num_height - 1)
                    self.constraints.append(DistanceConstraint(idx, idx+self.num_width, rest_length, stiffness))
        for i in range(self.num_height-1):
            for j in range(self.num_width-1):
                idx = i * self.num_width + j
                self.constraints.append(BendConstraint(idx, idx+1, idx+self.num_width+1, stiffness*0.3))

    def apply_external_force(self, force):
        for p in self.particles:
            if not p.fixed:
                p.force += force

    def update(self, dt, gravity=np.array([0,0,-9.81])):
        self.apply_external_force(gravity * dt)
        for p in self.particles:
            if not p.fixed:
                p.velocity += p.force * p.inv_mass * dt
                p.position += p.velocity * dt
                p.force = np.zeros(3)
                p.prev_position = p.position.copy()

    def apply_constraints(self, iterations=5):
        for _ in range(iterations):
            for c in self.constraints:
                corrections = c.solve(np.array([p.position for p in self.particles]), 0.0)
                for i, idx in enumerate(c.particles):
                    if not self.particles[idx].fixed:
                        self.particles[idx].position += corrections[i]
        for p in self.particles:
            if not p.fixed:
                p.velocity = (p.position - p.prev_position) / 0.016
