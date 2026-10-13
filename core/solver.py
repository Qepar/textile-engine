"""
Solver Classes for Textile Engine
Implements the Position-Based Dynamics solver for cloth simulation.
"""
import numpy as np
from .collision import detect_collisions, resolve_collisions
from .math_utils import norm

class SpatialGrid:
    """Spatial grid for efficient collision detection."""
    def __init__(self, cell_size=0.5):
        self.cell_size = cell_size
        self.grid = {}

    def clear(self):
        self.grid.clear()

    def add_particle(self, particle, index):
        cell = self._get_cell(particle.position)
        if cell not in self.grid:
            self.grid[cell] = []
        self.grid[cell].append(index)

    def _get_cell(self, position):
        x, y, z = position
        return (int(x / self.cell_size), int(y / self.cell_size), int(z / self.cell_size))

    def get_neighbors(self, particle, index):
        cell = self._get_cell(particle.position)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    neighbor_cell = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                    if neighbor_cell in self.grid:
                        for idx in self.grid[neighbor_cell]:
                            if idx != index:
                                neighbors.append(idx)
        return neighbors

def apply_internal_friction(cloth, friction_coeff=0.1, radius=0.15):
    """Apply internal friction between close particles."""
    particles = cloth.particles
    n = len(particles)
    for i in range(n):
        p1 = particles[i]
        for j in range(i + 1, n):
            p2 = particles[j]
            distance = norm(p1.position - p2.position)
            if distance < radius:
                relative_velocity = p2.velocity - p1.velocity
                friction_force = -friction_coeff * relative_velocity
                p1.force += friction_force
                p2.force -= friction_force

class PBDSolver:
    """Position-Based Dynamics solver for cloth simulation."""
    def __init__(self, cloth, ground_height=-1.0, friction=0.2, iterations=5, use_spatial_grid=False):
        self.cloth = cloth
        self.ground_height = ground_height
        self.friction = friction
        self.iterations = iterations
        self.use_spatial_grid = use_spatial_grid
        self.grid = SpatialGrid(cell_size=0.2) if use_spatial_grid else None

    def step(self, dt, gravity=np.array([0, 0, -9.81])):
        self.cloth.apply_external_force(gravity * dt * self.cloth.particles[0].mass)
        for p in self.cloth.particles:
            if not p.fixed:
                p.velocity += p.force * p.inv_mass * dt
                p.position += p.velocity * dt
                p.reset_force()
                p.prev_position = p.position.copy()

        for _ in range(self.iterations):
            positions = np.array([p.position for p in self.cloth.particles])
            for c in self.cloth.constraints:
                corrections = c.solve(positions, dt)
                for i, idx in enumerate(c.particles):
                    if not self.cloth.particles[idx].fixed:
                        self.cloth.particles[idx].position += corrections[i]

        if self.use_spatial_grid:
            self.grid.clear()
            for idx, p in enumerate(self.cloth.particles):
                self.grid.add_particle(p, idx)
            for idx, p in enumerate(self.cloth.particles):
                neighbors = self.grid.get_neighbors(p, idx)
                for neighbor_idx in neighbors:
                    p2 = self.cloth.particles[neighbor_idx]
                    distance = norm(p.position - p2.position)
                    min_distance = p.radius + p2.radius
                    if distance < min_distance:
                        normal = (p2.position - p.position) / distance if distance > 0 else np.array([0, 0, 1])
                        p.collision_normal = normal
                        p.collision_depth = min_distance - distance
        else:
            detect_collisions(self.cloth, self.ground_height)

        resolve_collisions(self.cloth, self.ground_height, self.friction)
        apply_internal_friction(self.cloth)

        for p in self.cloth.particles:
            if not p.fixed:
                p.velocity = (p.position - p.prev_position) / dt
