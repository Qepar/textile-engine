import numpy as np

def norm(v):
    """Calcule la norme d'un vecteur."""
    return np.linalg.norm(v)

def normalize(v):
    """Normalise un vecteur."""
    n = norm(v)
    return v / n if n > 0 else np.zeros_like(v)

def clamp(value, min_val, max_val):
    """Limite une valeur entre min_val et max_val."""
    return max(min_val, min(max_val, value))
