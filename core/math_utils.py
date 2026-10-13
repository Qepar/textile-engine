"""
Mathematical Utilities for Textile Engine
Provides common vector and matrix operations.
"""
import numpy as np

def norm(v):
    """Calculate the norm (length) of a vector."""
    return np.linalg.norm(v)

def normalize(v):
    """Normalize a vector to unit length."""
    n = norm(v)
    return v / n if n > 0 else np.zeros_like(v)

def clamp(value, min_val, max_val):
    """Clamp a value between minimum and maximum."""
    return max(min_val, min(max_val, value))
