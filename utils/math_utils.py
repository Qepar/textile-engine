import numpy as np

def norm(v):
    return np.linalg.norm(v)

def normalize(v):
    n = norm(v)
    return v / n if n > 0 else np.zeros_like(v)
