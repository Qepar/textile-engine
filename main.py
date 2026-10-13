from core.cloth import Cloth
from core.solver import PBDSolver
from rendering.opengl_renderer import OpenGLRenderer

def main():
    # Créer un textile (2m x 2m, 20x20 particules)
    cloth = Cloth(width=2.0, height=2.0, num_width=20, num_height=20, mass=0.05, stiffness=20.0)

    # Initialiser le renderer
    renderer = OpenGLRenderer(width=800, height=600, title="Textile Engine - Original PBD")
    renderer.set_cloth(cloth)

    # Lancer la simulation
    renderer.run()

if __name__ == "__main__":
    main()
