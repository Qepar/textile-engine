"""
OpenGL Renderer for Textile Engine
Handles 3D rendering using PyOpenGL.
"""
import sys
import numpy as np
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from .camera import Camera
from core.math_utils import norm

class OpenGLRenderer:
    def __init__(self, width=800, height=600, title="Textile Engine"):
        self.width = width
        self.height = height
        self.title = title
        self.cloth = None
        self.camera = Camera(position=np.array([0, 0, 5]))
        self.show_particles = False
        self.show_wireframe = False

    def set_cloth(self, cloth):
        self.cloth = cloth

    def init_gl(self):
        glClearColor(0.1, 0.1, 0.1, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def draw_cloth(self):
        if self.cloth is None:
            return
        if self.show_wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glColor3f(0.8, 0.2, 0.2)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glBegin(GL_TRIANGLES)
        for i in range(self.cloth.num_height - 1):
            for j in range(self.cloth.num_width - 1):
                idx = i * self.cloth.num_width + j
                p1 = self.cloth.particles[idx].position
                p2 = self.cloth.particles[idx + 1].position
                p3 = self.cloth.particles[idx + self.cloth.num_width].position
                p4 = self.cloth.particles[idx + self.cloth.num_width + 1].position
                normal = np.cross(p2 - p1, p3 - p1)
                normal = normal / norm(normal) if norm(normal) > 0 else np.array([0, 0, 1])
                glNormal3fv(normal)
                glColor3f(0.8, 0.2, 0.2)
                glVertex3fv(p1)
                glVertex3fv(p2)
                glVertex3fv(p3)
                glNormal3fv(normal)
                glVertex3fv(p2)
                glVertex3fv(p4)
                glVertex3fv(p3)
        glEnd()
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        if self.show_particles:
            glPointSize(3.0)
            glBegin(GL_POINTS)
            for p in self.cloth.particles:
                if p.fixed:
                    glColor3f(0, 1, 0)
                else:
                    glColor3f(1, 1, 0)
                glVertex3fv(p.position)
            glEnd()

    def draw_ground(self):
        glBegin(GL_QUADS)
        glNormal3f(0, 0, 1)
        glColor3f(0.2, 0.6, 0.2)
        glVertex3f(-10, -10, -1)
        glVertex3f(10, -10, -1)
        glVertex3f(10, 10, -1)
        glVertex3f(-10, 10, -1)
        glEnd()

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport(0, 0, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.camera.apply()
        self.draw_ground()
        self.draw_cloth()
        glutSwapBuffers()

    def update(self):
        if self.cloth:
            from core.solver import PBDSolver
            dt = 0.016
            solver = PBDSolver(self.cloth, ground_height=-1.0, friction=0.2)
            solver.step(dt)
        self.render()

    def reshape(self, width, height):
        self.width = width
        self.height = height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def keyboard(self, key, x, y):
        if key == b'\x1b':
            sys.exit(0)
        elif key == b' ':
            if self.cloth:
                self.cloth.reset()
        elif key == b'w':
            self.show_wireframe = not self.show_wireframe
        elif key == b'p':
            self.show_particles = not self.show_particles

    def special_key(self, key, x, y):
        if key == GLUT_KEY_UP:
            self.camera.move_forward(0.2)
        elif key == GLUT_KEY_DOWN:
            self.camera.move_forward(-0.2)
        elif key == GLUT_KEY_LEFT:
            self.camera.move_right(-0.2)
        elif key == GLUT_KEY_RIGHT:
            self.camera.move_right(0.2)

    def run(self):
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
        glutMainLoop()
