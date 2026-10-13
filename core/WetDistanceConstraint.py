class Particle:
    def __init__(self, position, velocity=np.zeros(3), mass=1.0, fixed=False, radius=0.05, dampness=0.0):
        # ... (champs existants)
        self.dampness = dampness  # Niveau d'humidité (0.0 = sec, 1.0 = trempé)
        self.temperature = 20.0   # Température en °C

class WetDistanceConstraint(DistanceConstraint):
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness=1.0):
        super().__init__(p1_idx, p2_idx, rest_length, stiffness)

    def solve(self, positions, dt):
        p1, p2 = positions[self.particles[0]], positions[self.particles[1]]
        # Réduire la raideur si le tissu est mouillé
        avg_dampness = (p1.dampness + p2.dampness) / 2
        effective_stiffness = self.stiffness * (1.0 - 0.5 * avg_dampness)  # 50% moins raide si trempé

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
