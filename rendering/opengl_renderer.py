import numpy as np
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from pygame.locals import *

class OpenGLRenderer:
    def __init__(self, width=800, height=600, title="Textile Engine"):
        self.width = width
        self.height = height
        self.title = title
        self.cloth = None
        self.camera = Camera()
        self.running = False

    def init_gl(self):
        """Initialise OpenGL."""
        glClearColor(0.1, 0.1, 0.1, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)

        # Configuration de la lumière
        light_pos = [1.0, 1.0, 1.0, 0.0]  # Position directionnelle
        light_ambient = [0.2, 0.2, 0.2, 1.0]
        light_diffuse = [0.8, 0.8, 0.8, 1.0]
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)

    def set_cloth(self, cloth):
        """Associe un objet Cloth au renderer."""
        self.cloth = cloth

    def draw_cloth(self):
        """Dessine le textile."""
        if self.cloth is None:
            return

        glBegin(GL_TRIANGLES)
        for i in range(self.cloth.num_height - 1):
            for j in range(self.cloth.num_width - 1):
                # Indices des 4 coins du carré
                idx = i * self.cloth.num_width + j
                p1 = self.cloth.particles[idx].position
                p2 = self.cloth.particles[idx + 1].position
                p3 = self.cloth.particles[idx + self.cloth.num_width].position
                p4 = self.cloth.particles[idx + self.cloth.num_width + 1].position

                # Calculer la normale (pour l'éclairage)
                normal = np.cross(p2 - p1, p3 - p1)
                normal = normal / np.linalg.norm(normal)

                glNormal3fv(normal)
                glColor3f(0.8, 0.2, 0.2)  # Rouge
                glVertex3fv(p1)
                glVertex3fv(p2)
                glVertex3fv(p3)

                glNormal3fv(normal)
                glVertex3fv(p2)
                glVertex3fv(p4)
                glVertex3fv(p3)
        glEnd()

        # Dessiner les particules (pour le debug)
        glPointSize(3.0)
        glBegin(GL_POINTS)
        for p in self.cloth.particles:
            if p.fixed:
                glColor3f(0, 1, 0)  # Vert pour les particules fixes
            else:
                glColor3f(1, 1, 0)  # Jaune pour les autres
            glVertex3fv(p.position)
        glEnd()

    def draw_ground(self):
        """Dessine le sol."""
        glBegin(GL_QUADS)
        glNormal3f(0, 0, 1)
        glColor3f(0.2, 0.6, 0.2)  # Vert
        glVertex3f(-10, -10, -1)
        glVertex3f(10, -10, -1)
        glVertex3f(10, 10, -1)
        glVertex3f(-10, 10, -1)
        glEnd()

    def render(self):
        """Affiche la scène."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Appliquer la caméra
        self.camera.apply()

        # Dessiner le sol
        self.draw_ground()

        # Dessiner le textile
        self.draw_cloth()

        glutSwapBuffers()

    def run(self):
        """Lance la boucle principale."""
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.width, self.height)
        glutCreateWindow(self.title.encode())
        self.init_gl()

        glutDisplayFunc(self.render)
        glutIdleFunc(self.update)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyboard)
        glutSpecialFunc(self.special_key)

        self.running = True
        glutMainLoop()

    def update(self):
        """Met à jour la simulation."""
        if self.cloth and self.running:
            import time
            dt = 0.016  # ~60 FPS
            solver = PBDSolver(self.cloth, ground_height=-1.0, friction=0.2)
            solver.step(dt)
            self.render()

    def reshape(self, width, height):
        """Gère le redimensionnement de la fenêtre."""
        self.width = width
        self.height = height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def keyboard(self, key, x, y):
        """Gère les touches du clavier."""
        if key == b'\x1b':  # Échap
            self.running = False
            glutLeaveMainLoop()

    def special_key(self, key, x, y):
        """Gère les touches spéciales (flèches, etc.)."""
        if key == GLUT_KEY_UP:
            self.camera.move_forward(0.1)
        elif key == GLUT_KEY_DOWN:
            self.camera.move_forward(-0.1)
        elif key == GLUT_KEY_LEFT:
            self.camera.move_right(-0.1)
        elif key == GLUT_KEY_RIGHT:
            self.camera.move_right(0.1)

class Camera:
    def __init__(self, position=np.array([0, 0, 5]), target=np.array([0, 0, 0]), up=np.array([0, 1, 0])):
        self.position = position
        self.target = target
        self.up = up
        self.yaw = 0.0
        self.pitch = 0.0

    def apply(self):
        """Applique la vue de la caméra."""
        gluLookAt(
            self.position[0], self.position[1], self.position[2],
            self.target[0], self.target[1], self.target[2],
            self.up[0], self.up[1], self.up[2]
        )

    def move_forward(self, distance):
        """Déplace la caméra vers l'avant/arrière."""
        direction = self.target - self.position
        direction = direction / np.linalg.norm(direction)
        self.position += direction * distance
        self.target += direction * distance

    def move_right(self, distance):
        """Déplace la caméra à gauche/droite."""
        direction = self.target - self.position
        right = np.cross(direction, self.up)
        right = right / np.linalg.norm(right)
        self.position += right * distance
        self.target += right * distance
