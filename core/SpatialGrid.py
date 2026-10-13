class SpatialGrid:
    def __init__(self, cell_size=0.5):
        self.cell_size = cell_size
        self.grid = {}  # Dictionnaire : clé = (x, y, z), valeur = liste d'indices de particules

    def add_particle(self, particle, index):
        """Ajoute une particule à la grille."""
        cell = self._get_cell(particle.position)
        if cell not in self.grid:
            self.grid[cell] = []
        self.grid[cell].append(index)

    def _get_cell(self, position):
        """Retourne la cellule correspondant à une position."""
        x, y, z = position
        return (
            int(x / self.cell_size),
            int(y / self.cell_size),
            int(z / self.cell_size)
        )

    def get_neighbors(self, particle, index):
        """Retourne les indices des particules voisines (même cellule ou cellules adjacentes)."""
        cell = self._get_cell(particle.position)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    neighbor_cell = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                    if neighbor_cell in self.grid:
                        for idx in self.grid[neighbor_cell]:
                            if idx != index:  # Éviter la particule elle-même
                                neighbors.append(idx)
        return neighbors

def detect_collisions_with_grid(cloth, grid):
    """Détecte les collisions en utilisant une grille spatiale."""
    grid.clear()
    for idx, p in enumerate(cloth.particles):
        grid.add_particle(p, idx)

    for idx, p in enumerate(cloth.particles):
        neighbors = grid.get_neighbors(p, idx)
        for neighbor_idx in neighbors:
            p2 = cloth.particles[neighbor_idx]
            if norm(p.position - p2.position) < p.radius + p2.radius:
                # Collision détectée
                normal = normalize(p2.position - p.position)
                p.collision_normal = normal
                p.collision_depth = p.radius + p2.radius - norm(p.position - p2.position)
