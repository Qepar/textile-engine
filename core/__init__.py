"""
Textile Engine - Core Module
"""
from .particle import Particle
from .cloth import Cloth
from .constraints import (
    Constraint,
    DistanceConstraint,
    BendConstraint,
    ShearConstraint,
    AnisotropicDistanceConstraint,
    WetDistanceConstraint,
)
from .solver import PBDSolver, SpatialGrid
from .collision import detect_collisions, resolve_collisions
from utils.math_utils import norm, normalize
