class AnisotropicDistanceConstraint(DistanceConstraint):
    def __init__(self, p1_idx, p2_idx, rest_length, stiffness_x, stiffness_y, direction):
        super().__init__(p1_idx, p2_idx, rest_length)
        self.stiffness_x = stiffness_x  # Raideur dans la direction x
        self.stiffness_y = stiffness_y  # Raideur dans la direction y
        self.direction = direction      # Direction principale du tissu

    def solve(self, positions, dt):
        p1, p2 = positions[self.particles[0]], positions[self.particles[1]]
        delta = p2 - p1
        current_length = norm(delta)

        # Projeter delta sur la direction principale
        projection = np.dot(delta, self.direction)
        parallel_component = projection * self.direction
        perpendicular_component = delta - parallel_component

        # Appliquer des raideurs différentes selon la direction
        parallel_correction = (norm(parallel_component) - self.rest_length) / norm(parallel_component) if norm(parallel_component) > 0 else 0
        perpendicular_correction = (norm(perpendicular_component) - 0) / norm(perpendicular_component) if norm(perpendicular_component) > 0 else 0

        correction = (
            parallel_correction * self.stiffness_x * parallel_component +
            perpendicular_correction * self.stiffness_y * perpendicular_component
        ) * 0.5

        return np.array([correction, -correction])
