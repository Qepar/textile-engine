def apply_internal_friction(cloth, friction_coeff=0.1, radius=0.1):
    """Applique des frottements entre particules proches (simule les frottements internes du tissu)."""
    particles = cloth.particles
    for i, p1 in enumerate(particles):
        for j, p2 in enumerate(particles[i+1:], i+1):
            if norm(p1.position - p2.position) < radius:
                # Calculer la force de frottement (proportionnelle à la différence de vitesse)
                relative_velocity = p2.velocity - p1.velocity
                friction_force = -friction_coeff * relative_velocity
                p1.force += friction_force
                p2.force -= friction_force
