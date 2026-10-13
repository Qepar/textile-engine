class WrinkleConstraint(Constraint):
    def __init__(self, p1_idx, p2_idx, p3_idx, wrinkle_map, stiffness=0.5):
        super().__init__([p1_idx, p2_idx, p3_idx], stiffness)
        self.wrinkle_map = wrinkle_map  # Carte des plis (ex: un tableau 2D)

    def solve(self, positions, dt):
        p1, p2, p3 = [positions[i] for i in self.particles]
        v1 = p1 - p2
        v2 = p3 - p2
        current_angle = np.arccos(np.clip(np.dot(v1, v2) / (norm(v1) * norm(v2)), -1.0, 1.0))

        # Si l'angle est inférieur à un seuil, on considère qu'il y a un pli
        if current_angle < np.pi * 0.8:  # Seuil à 144°
            self.wrinkle_map[self.particles[1]] = True  # Marquer p2 comme étant dans un pli

            # Appliquer une correction pour stabiliser le pli
            rest_angle = np.pi * 0.7  # Angle de repos pour un pli
            delta_angle = current_angle - rest_angle
            axis = np.cross(v1, v2)
            if norm(axis) > 0:
                axis = normalize(axis)
                theta = delta_angle * self.stiffness
                K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
                correction = np.dot(R, p2 - (p1 + p3) / 2) - (p2 - (p1 + p3) / 2)
                return np.array([np.zeros(3), correction, np.zeros(3)])
        else:
            self.wrinkle_map[self.particles[1]] = False
        return np.zeros((3, 3))
