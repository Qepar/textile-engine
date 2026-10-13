import numpy as np
from .math_utils import norm

def detect_collisions(cloth, ground_height):
    """Détecte les collisions avec le sol."""
    for p in cloth.particles:
        if p.position[2] < ground_height:
            p.collision_normal = np.array([0, 0, 1])  # Normale du sol (vers le haut)
            p.collision_depth = ground_height - p.position[2]
        else:
            p.collision_normal = None
            p.collision_depth = 0.0

def resolve_collisions(cloth, ground_height, friction):
    """Résout les collisions avec le sol."""
    for p in cloth.particles:
        if p.collision_normal is not None:
            # Correction de position (pour éviter l'interpénétration)
            p.position += p.collision_normal * p.collision_depth

            # Réponse élastique (rebond)
            velocity_along_normal = np.dot(p.velocity, p.collision_normal)
            if velocity_along_normal < 0:  # Si la particule descend
                p.velocity -= (1.0 + 0.5) * velocity_along_normal * p.collision_normal  # Coefficient de rebond = 0.5

            # Frottement (réduit la vitesse tangentielle)
            tangent_velocity = p.velocity - velocity_along_normal * p.collision_normal
            if norm(tangent_velocity) > 0:
                tangent_velocity *= max(0, 1 - friction * norm(tangent_velocity))
                p.velocity = velocity_along_normal * p.collision_normal + tangent_velocity
