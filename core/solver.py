import numpy as np
from .collision import detect_collisions, resolve_collisions

class PBDSolver:
    def __init__(self, cloth, ground_height=-1.0, friction=0.2):
        self.cloth = cloth
        self.ground_height = ground_height
        self.friction = friction

    def step(self, dt, gravity=np.array([0, 0, -9.81])):
        """Effectue une étape de simulation."""
        # 1. Appliquer les forces externes (gravité, vent, etc.)
        self.cloth.apply_external_force(gravity * dt)

        # 2. Prédire les positions
        for p in self.cloth.particles:
            if not p.fixed:
                p.velocity += p.force * p.inv_mass * dt
                p.position += p.velocity * dt
                p.force = np.zeros(3)

        # 3. Appliquer les contraintes
        for _ in range(5):  # Itérations pour la stabilité
            for constraint in self.cloth.constraints:
                corrections = constraint.solve(
                    np.array([p.position for p in self.cloth.particles]),
                    dt
                )
                for i, idx in enumerate(constraint.particles):
                    if not self.cloth.particles[idx].fixed:
                        self.cloth.particles[idx].position += corrections[i]

        # 4. Détecter et résoudre les collisions
        detect_collisions(self.cloth, self.ground_height)
        resolve_collisions(self.cloth, self.ground_height, self.friction)

        # 5. Mettre à jour les vitesses
        for p in self.cloth.particles:
            if not p.fixed:
                p.velocity = (p.position - p.prev_position) / dt
            p.prev_position = p.position.copy()
