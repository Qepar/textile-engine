"""
Constraint Classes for Textile Engine
Implements various constraints for cloth simulation.
"""
import numpy as np
from .math_utils import norm, normalize

class Constraint:
    """Base class for all constraints."""
    def __init__(self, particles, stiffness=1.0):
        self.particles = particles
        self.stiffness = stiffness

    def solve(self, positions, dt):
        """Apply the constraint and return position corrections."""
        raise NotImplementedError

class DistanceConstraint(Constraint):
    """Distance constraint between two particles (spring-like behavior)."""
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness=1.0):
        super().__init__([p1_idx, p2_idx], stiffness)
        self.rest_length = rest_length

    def solve(self, positions, dt):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        delta = p2 - p1
        current_length = norm(delta)
        if current_length == 0:
            return np.zeros((2, 3))
        correction = (current_length - self.rest_length) / current_length
        delta_correction = 0.5 * self.stiffness * correction * delta
        return np.array([delta_correction, -delta_correction])

class BendConstraint(Constraint):
    """Bend constraint for three particles to control folding behavior."""
    def __init__(self, p1_idx, p2_idx, p3_idx, stiffness=0.5):
        super().__init__([p1_idx, p2_idx, p3_idx], stiffness)
        self.rest_angle = np.pi

    def solve(self, positions, dt):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        p3 = positions[self.particles[2]]
        v1 = p1 - p2
        v2 = p3 - p2
        current_length_v1 = norm(v1)
        current_length_v2 = norm(v2)
        if current_length_v1 == 0 or current_length_v2 == 0:
            return np.zeros((3, 3))
        cos_angle = np.dot(v1, v2) / (current_length_v1 * current_length_v2)
        current_angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        delta_angle = current_angle - self.rest_angle
        axis = np.cross(v1, v2)
        if norm(axis) == 0:
            return np.zeros((3, 3))
        axis = normalize(axis)
        theta = delta_angle * self.stiffness
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        correction = np.dot(R, p2 - (p1 + p3) / 2) - (p2 - (p1 + p3) / 2)
        return np.array([np.zeros(3), correction, np.zeros(3)])

class ShearConstraint(Constraint):
    """Shear constraint for four particles to control diagonal stiffness."""
    def __init__(self, p1_idx, p2_idx, p3_idx, p4_idx, stiffness=0.8):
        super().__init__([p1_idx, p2_idx, p3_idx, p4_idx], stiffness)
        self.rest_length = 0.0

    def set_rest_length(self, positions):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        p3 = positions[self.particles[2]]
        p4 = positions[self.particles[3]]
        d1 = p3 - p1
        d2 = p4 - p2
        self.rest_length = norm(d1 - d2)

    def solve(self, positions, dt):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        p3 = positions[self.particles[2]]
        p4 = positions[self.particles[3]]
        d1 = p3 - p1
        d2 = p4 - p2
        current_length = norm(d1 - d2)
        if current_length == 0:
            return np.zeros((4, 3))
        correction = (current_length - self.rest_length) / current_length
        delta_correction = 0.25 * self.stiffness * correction * (d1 - d2)
        return np.array([delta_correction, delta_correction, -delta_correction, -delta_correction])

class AnisotropicDistanceConstraint(DistanceConstraint):
    """Anisotropic distance constraint for direction-dependent stiffness."""
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness_x=1.0, stiffness_y=0.5, direction=np.array([1, 0, 0])):
        super().__init__(p1_idx, p2_idx, rest_length, 1.0)
        self.stiffness_x = stiffness_x
        self.stiffness_y = stiffness_y
        self.direction = normalize(direction)

    def solve(self, positions, dt):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        delta = p2 - p1
        current_length = norm(delta)
        if current_length == 0:
            return np.zeros((2, 3))
        projection = np.dot(delta, self.direction)
        parallel_component = projection * self.direction
        perpendicular_component = delta - parallel_component
        parallel_correction = (norm(parallel_component) - self.rest_length) / norm(parallel_component) if norm(parallel_component) > 0 else 0
        perpendicular_correction = norm(perpendicular_component) / norm(perpendicular_component) if norm(perpendicular_component) > 0 else 0
        correction = (
            parallel_correction * self.stiffness_x * parallel_component +
            perpendicular_correction * self.stiffness_y * perpendicular_component
        ) * 0.5
        inv_mass1 = 1.0 if positions[self.particles[0]].mass == 0 else positions[self.particles[0]].inv_mass
        inv_mass2 = 1.0 if positions[self.particles[1]].mass == 0 else positions[self.particles[1]].inv_mass
        total_inv_mass = inv_mass1 + inv_mass2
        if total_inv_mass > 0:
            correction *= 1.0 / total_inv_mass
        return np.array([correction, -correction])

class WetDistanceConstraint(DistanceConstraint):
    """Distance constraint that accounts for fabric dampness."""
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness=1.0):
        super().__init__(p1_idx, p2_idx, rest_length, stiffness)

    def solve(self, positions, dt):
        p1 = positions[self.particles[0]]
        p2 = positions[self.particles[1]]
        avg_dampness = (p1.dampness + p2.dampness) / 2
        effective_stiffness = self.stiffness * (1.0 - 0.5 * avg_dampness)
        delta = p2 - p1
        current_length = norm(delta)
        if current_length == 0:
            return np.zeros((2, 3))
        correction = (current_length - self.rest_length) / current_length
        delta_correction = 0.5 * effective_stiffness * correction * delta
        inv_mass1 = 1.0 if p1.mass == 0 else p1.inv_mass
        inv_mass2 = 1.0 if p2.mass == 0 else p2.inv_mass
        total_inv_mass = inv_mass1 + inv_mass2
        if total_inv_mass > 0:
            delta_correction *= effective_stiffness / total_inv_mass
        return np.array([delta_correction, -delta_correction])
