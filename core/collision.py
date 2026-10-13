"""
Collision Detection and Resolution for Textile Engine
"""
import numpy as np
from .math_utils import norm, normalize

def detect_collisions(cloth, ground_height=-1.0):
    """Detect collisions with the ground for all particles in the cloth."""
    for p in cloth.particles:
        if p.position[2] < ground_height:
            p.collision_normal = np.array([0, 0, 1], dtype=np.float64)
            p.collision_depth = ground_height - p.position[2]
        else:
            p.collision_normal = None
            p.collision_depth = 0.0

def resolve_collisions(cloth, ground_height=-1.0, friction=0.2):
    """Resolve collisions with the ground."""
    for p in cloth.particles:
        if p.collision_normal is not None:
            p.position += p.collision_normal * p.collision_depth
            velocity_along_normal = np.dot(p.velocity, p.collision_normal)
            if velocity_along_normal < 0:
                restitution = 0.5
                p.velocity -= (1.0 + restitution) * velocity_along_normal * p.collision_normal
            tangent_velocity = p.velocity - velocity_along_normal * p.collision_normal
            if norm(tangent_velocity) > 0:
                tangent_velocity *= max(0, 1 - friction * norm(tangent_velocity))
                p.velocity = velocity_along_normal * p.collision_normal + tangent_velocity
