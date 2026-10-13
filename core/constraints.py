import numpy as np
from .math_utils import norm, normalize

class Constraint:
    """Classe de base pour toutes les contraintes."""
    def __init__(self, particles, stiffness=1.0):
        self.particles = particles  # Liste des indices des particules concernées
        self.stiffness = stiffness    # Raideur de la contrainte

    def solve(self, positions, dt):
        """Applique la contrainte et retourne les corrections de position."""
        raise NotImplementedError

class DistanceConstraint(Constraint):
    """Contrainte de distance (ressort entre 2 particules)."""
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness=1.0):
        super().__init__([p1_idx, p2_idx], stiffness)
        self.rest_length = rest_length  # Longueur au repos

    def solve(self, positions, dt):
        p1, p2 = positions[self.particles[0]], positions[self.particles[1]]
        delta = p2 - p1
        current_length = norm(delta)
        if current_length == 0:
            return np.zeros((2, 3))

        # Correction PBD : déplace les particules pour respecter la longueur au repos
        correction = (current_length - self.rest_length) / current_length
        delta_correction = 0.5 * self.stiffness * correction * delta

        # Appliquer la correction (en tenant compte de la masse inverse)
        inv_mass1 = 1.0 if positions[self.particles[0]].mass == 0 else positions[self.particles[0]].inv_mass
        inv_mass2 = 1.0 if positions[self.particles[1]].mass == 0 else positions[self.particles[1]].inv_mass
        total_inv_mass = inv_mass1 + inv_mass2

        if total_inv_mass > 0:
            delta_correction *= self.stiffness / total_inv_mass

        return np.array([
            delta_correction,
            -delta_correction
        ])

class BendConstraint(Constraint):
    """Contrainte de flexion (pour éviter les plis trop aigus)."""
    def __init__(self, p1_idx, p2_idx, p3_idx, stiffness=0.5):
        super().__init__([p1_idx, p2_idx, p3_idx], stiffness)

    def solve(self, positions, dt):
        p1, p2, p3 = positions[self.particles[0]], positions[self.particles[1]], positions[self.particles[2]]
        v1 = p1 - p2
        v2 = p3 - p2
        current_angle = np.arccos(np.clip(np.dot(v1, v2) / (norm(v1) * norm(v2)), -1.0, 1.0))

        # Angle de repos (par exemple, 180° pour un tissu plat)
        rest_angle = np.pi
        delta_angle = current_angle - rest_angle

        # Correction : faire tourner p2 pour réduire l'angle
        if norm(v1) * norm(v2) == 0:
            return np.zeros((3, 3))

        # Calculer l'axe de rotation (produit vectoriel)
        axis = np.cross(v1, v2)
        if norm(axis) == 0:
            return np.zeros((3, 3))

        axis = normalize(axis)
        theta = delta_angle * self.stiffness  # Angle de correction

        # Matrice de rotation (formule de Rodrigues)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

        # Appliquer la rotation à p2
        correction = np.dot(R, p2 - (p1 + p3) / 2) - (p2 - (p1 + p3) / 2)

        return np.array([
            np.zeros(3),  # p1 ne bouge pas
            correction,    # p2 est corrigé
            np.zeros(3)   # p3 ne bouge pas
        ])

class ShearConstraint(Constraint):
    """Contrainte de cisaillement (pour la rigidité en diagonale)."""
    def __init__(self, p1_idx, p2_idx, p3_idx, p4_idx, stiffness=0.8):
        super().__init__([p1_idx, p2_idx, p3_idx, p4_idx], stiffness)

    def solve(self, positions, dt):
        p1, p2, p3, p4 = [positions[i] for i in self.particles]
        # Calculer les vecteurs diagonaux
        d1 = p3 - p1
        d2 = p4 - p2
        current_length = norm(d1 - d2)

        # Longueur au repos (distance entre les diagonales dans le maillage initial)
        rest_length = 0.0  # À calculer lors de l'initialisation

        if current_length == 0:
            return np.zeros((4, 3))

        correction = (current_length - rest_length) / current_length
        delta_correction = 0.25 * self.stiffness * correction * (d1 - d2)

        return np.array([
            delta_correction,
            delta_correction,
            -delta_correction,
            -delta_correction
        ])
