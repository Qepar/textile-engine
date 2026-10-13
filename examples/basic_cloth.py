from core.cloth import Cloth
from rendering.opengl_renderer import OpenGLRenderer

def main():
    cloth = Cloth(width=2.0, height=2.0, num_width=20, num_height=20, mass=0.05, stiffness=20.0)
    renderer = OpenGLRenderer(width=800, height=600, title="Textile Engine - Basic Cloth")
    renderer.set_cloth(cloth)
    renderer.show_particles = True
    renderer.run()

if __name__ == "__main__":
    main()
