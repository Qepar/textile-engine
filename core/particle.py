"""
Particle Class for Textile Engine
Represents a single point in the cloth simulation with physical properties.
"""
import numpy as np

class Particle:
    """
    A particle in the cloth simulation.

    Attributes:
        position (np.ndarray): 3D position (x, y, z)
        velocity (np.ndarray): 3D velocity vector
        force (np.ndarray): Accumulated force vector
        mass (float): Particle mass
        inv_mass (float): Inverse mass (1/mass)
        fixed (bool): Whether the particle is fixed in space
        radius (float): Particle radius for collision detection
        prev_position (np.ndarray): Position from previous frame
        collision_normal (np.ndarray or None): Normal vector for collision response
        collision_depth (float): Depth of collision penetration
        dampness (float): Wetness level (0.0 = dry, 1.0 = soaked)
        temperature (float): Temperature in Celsius
    """
    def __init__(self, position, velocity=np.zeros(3), mass=1.0, fixed=False, radius=0.05, dampness=0.0, temperature=20.0):
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
        self.dampness = dampness
        self.temperature = temperature

    def reset_force(self):
        """Reset accumulated forces."""
        self.force = np.zeros(3, dtype=np.float64)

    def apply_force(self, force):
        """Apply an external force to the particle."""
        if not self.fixed:
            self.force += force
