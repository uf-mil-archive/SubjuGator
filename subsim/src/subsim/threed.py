from __future__ import division

import math
import sys
import random

import numpy
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame

import roslib
roslib.load_manifest('subsim')
import rospy
import tf
from tf import transformations
from sensor_msgs.msg import Image, CameraInfo

from vector import v, V

@apply
class GLMatrix(object):
    def __enter__(self):
        glPushMatrix()
    
    def __exit__(self, type, value, traceback):
        glPopMatrix()

class DisplayList(object):
    def __init__(self):
        self.sl = glGenLists(1)
    
    def __del__(self):
        glDeleteLists(self.sl, 1)
    
    def __enter__(self):
        glNewList(self.sl, GL_COMPILE)
    
    def __exit__(self, type, value, traceback):
        glEndList()
    
    def __call__(self):
        glCallList(self.sl)

def display_list(callable):
    cache = {}
    def x(*args):
        if args not in cache:
            dl = DisplayList()
            with dl:
                callable(*args)
            cache[args] = dl
        cache[args]()
    return x

def angleaxis_matrix(angle, (x, y, z)):
    s = math.sin(angle)
    c = math.cos(angle)
    return numpy.array([
        [(1-c)*x*x + c,   (1-c)*y*x - z*s, (1-c)*z*x + y*s, 0],
        [(1-c)*x*y + z*s, (1-c)*y*y + c,   (1-c)*z*y - x*s, 0],
        [(1-c)*x*z - y*s, (1-c)*y*z + x*s, (1-c)*z*z + c,   0],
        [0,               0,                0,              1],
    ])

def euler_matrix(yaw, pitch, roll):
    return numpy.dot(angleaxis_matrix(roll, (1, 0, 0)), numpy.dot(angleaxis_matrix(pitch, (0, 1, 0)), angleaxis_matrix(yaw, (0, 0, 1))))

def rotate_vec(vec, m):
    x = numpy.dot((vec[0], vec[1], vec[2], 1), m)
    return V(x[:3])/x[3]

def mesh_from_obj(file):
    vertices = []
    texcoords = []
    normals = []
    indices = []
    for line in file:
        line = line.strip()
        if '#' in line:
            line = line[:line.index("#")]
        if not line: continue
        line = line.split(' ')
        if line[0] == "v":
            vertices.append(V(float(x) for x in line[1:]))
        if line[0] == "vt":
            texcoords.append(V(float(x) for x in line[1:]))
        if line[0] == "vn":
            normals.append(V(float(x) for x in line[1:]))
        elif line[0] == "f":
            indices.append(tuple(tuple(int(y)-1 if y else None for y in x.split('/')) for x in line[1:]))
    return Mesh(vertices, texcoords, normals, indices)

class Mesh(object):
    def __init__(self, vertices, texcoords, normals, indices):
        self.vertices = vertices
        self.texcoords = texcoords
        self.normals = normals
        self.indices = indices
    
    @display_list
    def draw(self):
        glBegin(GL_TRIANGLES)
        for triangle in self.indices:
            for vert_index, tex_index, normal_index in triangle:
                if tex_index is not None:
                    glTexCoord2f(*self.texcoords[tex_index])
                if normal_index is not None:
                    glNormal3f(*self.normals[normal_index])
                glVertex3f(*self.vertices[vert_index])
        glEnd()
    
    @property
    def ode_trimeshdata(self):
        import ode
        x = ode.TriMeshData()
        x.build(self.vertices, [[p[0] for p in t] for t in self.indices])
        return x


def rotate_to_body(body, inv=False):
    R = body.getRotation()
    p = body.getPosition()
    rot = [[R[0], R[3], R[6], 0.],
        [R[1], R[4], R[7], 0.],
        [R[2], R[5], R[8], 0.],
        [p[0], p[1], p[2], 1.]]
    if inv:
        rot = numpy.linalg.inv(rot)
    rot = list(rot[0])+list(rot[1])+list(rot[2])+list(rot[3])
    glMultMatrixd(rot)

class Sub(object):
    def __init__(self, body, is_killed_func):
        self.body = body
        self.is_killed_func = is_killed_func
        self.vectors = []
    
    def draw(self):
        glBegin(GL_LINES)
        glColor3f(1, 1, 1)
        glVertex3f(*self.body.getPosition())
        glVertex3f(*self.body.getPosition() + 20*v(0.241510, -0.022442, 0.405728))
        glEnd()
        with GLMatrix:
            rotate_to_body(self.body)
            
            q = gluNewQuadric()
            with GLMatrix:
                glTranslate(+.2, 0, 0)
                glColor3f(0, 1, 0)
                gluSphere(q, 0.5, 40, 20)
            with GLMatrix:
                glTranslate(-.2, 0, 0)
                glColor3f(1, 0, 0)
                gluSphere(q, 0.5, 40, 20)
            with GLMatrix:
                glTranslate(+.4, +.2, -.3)
                glColor3f(0, 0, 0)
                gluSphere(q, 0.1, 40, 20)
            with GLMatrix:
                glTranslate(+.4, -.2, -.3)
                glColor3f(0, 0, 0)
                gluSphere(q, 0.1, 40, 20)
            with GLMatrix:
                glTranslate(0, 0, -.5)
                if self.is_killed_func():
                    glColor3f(1, 0, 0)
                else:
                    glColor3f(0, 1, 0)
                gluSphere(q, 0.1, 20, 10)
            
            glDisable(GL_DEPTH_TEST)
            glBegin(GL_LINES)
            for start, end in self.vectors:
                glColor3f(0, 0, 0)
                glVertex3f(*start)
                glColor3f(1, 1, 1)
                glVertex3f(*end)
            glEnd()
            glEnable(GL_DEPTH_TEST)

class Buoys(object):
    def __init__(self, pos):
        self.buoys = [(pos + [random.gauss(0, 1) for i in xrange(3)], color) for color in [(1, 0, 0), (0, 1, 0), (1, 1, 0)]]
    
    def draw(self):
        q = gluNewQuadric()
        for pos, color in self.buoys:
            with GLMatrix:
                glTranslate(*pos)
                glColor3f(*color)
                gluSphere(q, .2, 30, 15)

class MeshDrawer(object):
    def __init__(self, mesh, color):
        self.mesh = mesh
        self.color = color
    
    def draw(self):
        glColor3f(*self.color)
        self.mesh.draw()

class VectorField(object):
    def __init__(self, func):
        self.func = func
    
    def draw(self):
        glBegin(GL_LINES)
        for x in xrange(-10, 10+1):
            for y in xrange(-10, 10+1):
                pos = v(x, y, 0)
                vel = self.func(pos)
                glColor3d(1, 0, 0)
                glVertex3f(*pos-vel/2)
                glColor3d(1, 1, 0)
                glVertex3f(*pos+vel/2)
        glEnd()

def perspective(fovy, aspect, zNear):
    f = 1/math.tan(math.radians(fovy)/2)
    glMultMatrixf([
        [f/aspect, 0, 0, 0],
        [ 0, f, 0, 0],
        [0, 0, -1, -1],
        [0, 0, -2*zNear, 0]
    ])

class Interface(object):
    def init(self):
        self.display_flags = pygame.DOUBLEBUF|pygame.OPENGL|pygame.RESIZABLE
        self.display = pygame.display.set_mode((700, 400), self.display_flags)
        self.clock = pygame.time.Clock()
        
        self.pitch = self.yaw = self.roll = 0
        self.grabbed = False
        
        self.fovy = 100
        
        self.t = 0
        self.pos = v(-10, 0, -2)
        
        self.objs = []
        self.objs.append(Buoys(v(10, 3, 4)))
        
        rospy.init_node('sim')
        self.image_pub = rospy.Publisher('/sim_camera/image_rect_color', Image)
        self.info_pub = rospy.Publisher('/sim_camera/camera_info', CameraInfo)
        self.tf_br = tf.TransformBroadcaster()
        
        self.sub_view = False
    
    def step(self):
        dt = self.clock.tick()/1000
        self.t += dt
        
        for event in pygame.event.get():
            if event.type == pygame.MOUSEMOTION:
                if self.grabbed:
                    self.yaw += -event.rel[0]/100
                    
                    self.pitch += event.rel[1]/100
                    # caps it to a quarter turn up or down
                    self.pitch = min(max(self.pitch, -math.pi/2), math.pi/2)
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    self.grabbed = not self.grabbed
                    pygame.event.set_grab(self.grabbed)
                    pygame.mouse.set_visible(not self.grabbed)
                
                elif event.key == pygame.K_RETURN:
                    self.sub_view = not self.sub_view
                
                elif event.key == pygame.K_q:
                    sys.exit()
            
            elif event.type == pygame.QUIT:
                sys.exit()
            
            elif event.type == pygame.VIDEORESIZE:
                self.display = pygame.display.set_mode(event.size, self.display_flags)
        
        rot_matrix = euler_matrix(self.yaw, self.pitch, self.roll)
        
        forward = rotate_vec(v(1,0,0), rot_matrix)
        left = rotate_vec(v(0,-1,0), rot_matrix)
        local_up = rotate_vec(v(0,0,-1), rot_matrix)
        
        keys = pygame.key.get_pressed()
        
        speed = 50 if keys[pygame.K_LSHIFT] else 5
        speed *= (.1 if keys[pygame.K_LCTRL] else 1)
        
        if keys[pygame.K_w]: self.pos += forward*dt*speed
        if keys[pygame.K_s]: self.pos += -forward*dt*speed
        if keys[pygame.K_a]: self.pos += left*dt*speed
        if keys[pygame.K_d]: self.pos += -left*dt*speed
        if keys[pygame.K_SPACE]: self.pos += v(0, 0, -1)*dt*speed
        if keys[pygame.K_c]: self.pos += v(0, 0, 1)*dt*speed
        
        glViewport(0, 0, self.display.get_width(), self.display.get_height())
        
        glClearColor(0, 0, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        perspective(self.fovy, self.display.get_width()/self.display.get_height(), 0.1)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # rotates into the FRD coordinate system
        glMultMatrixf([
            [ 0., 0.,-1., 0.],
            [ 1., 0., 0., 0.],
            [ 0.,-1., 0., 0.],
            [ 0., 0., 0., 1.]
        ])
        # after that, +x is forward, +y is right, and +z is down
        
        for obj in self.objs:
            if self.sub_view and isinstance(obj, Sub):
                glTranslate(-.4, -.2, +.3)
                rotate_to_body(obj.body, inv=True)
                pos = obj.body.getRelPointPos((.4, .2, -.3))
                break
        else:
            glMultMatrixf(rot_matrix.T)
            
            glTranslate(*-self.pos)
            pos = self.pos
        
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0, 0, 0, 1])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0, 0, 0, 1])
        glLightfv(GL_LIGHT0, GL_POSITION, [math.sin(self.t/5)*100, math.cos(self.t/5)*100, -100, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0, 0, 0, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [.5, .5, .5, 1])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [.5, .5, .5])
        
        self.draw(pos)
        
        #print 'start'
        x = glReadPixels(0, 0, self.display.get_width(), self.display.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, outputType=None)
        x = numpy.reshape(x, (self.display.get_height(), self.display.get_width(), 4))
        x = x[::-1]
        #print 'end', type(x)
        
        t = rospy.Time.now()
        
        for obj in self.objs:
            if not isinstance(obj, Sub): continue
            with GLMatrix:
                rotate_to_body(obj.body)
                glcamera_from_frdbody = glGetFloatv(GL_MODELVIEW_MATRIX).T
                camera_from_body = numpy.array([ # camera from glcamera
                    [1, 0, 0, 0],
                    [0,-1, 0, 0],
                    [0, 0,-1, 0],
                    [0, 0, 0, 1],
                ]).dot(glcamera_from_frdbody).dot(numpy.array([ # frdbody from body
                    [1, 0, 0, 0],
                    [0,-1, 0, 0],
                    [0, 0,-1, 0],
                    [0, 0, 0, 1],
                ]))
                body_from_camera = numpy.linalg.inv(camera_from_body)
                self.tf_br.sendTransform(transformations.translation_from_matrix(body_from_camera),
                             transformations.quaternion_from_matrix(body_from_camera),
                             t,
                             "/sim_camera",
                             "/base_link")
        
        
        msg = Image()
        msg.header.stamp = t
        msg.header.frame_id = '/sim_camera'
        msg.height = self.display.get_height()
        msg.width = self.display.get_width()
        msg.encoding = 'rgba8'
        msg.is_bigendian = 0
        msg.step = self.display.get_width() * 4
        msg.data = x.tostring()
        self.image_pub.publish(msg)
        
        msg = CameraInfo()
        msg.header.stamp = t
        msg.header.frame_id = '/sim_camera'
        msg.height = self.display.get_height()
        msg.width = self.display.get_width()
        f = 1/math.tan(math.radians(self.fovy)/2)*self.display.get_height()/2
        msg.P = [
            f, 0, self.display.get_width()/2-.5, 0,
            0, f, self.display.get_height()/2-.5, 0,
            0, 0, 1, 0,
        ]
        self.info_pub.publish(msg)
        
        
        pygame.display.flip()
    
    def draw(self, pos):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        
        for obj in self.objs:
            #if self.sub_view and isinstance(obj, Sub):
            #    continue
            obj.draw()
        
        # sun
        with GLMatrix:
            glTranslate(math.sin(self.t/5)*100, math.cos(self.t/5)*100, -100)
            q = gluNewQuadric()
            glDisable(GL_LIGHTING)
            glColor3f(1, 1, 1)
            gluSphere(q, 10, 20, 20)
            glEnable(GL_LIGHTING)
        
        # water
        glDisable(GL_LIGHTING)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glColor4f(0, 0, 0.5, 0.5)
        glVertex4f(0, 0, 0, 1)
        glVertex4f(-1, -1, 0, 0)
        glVertex4f(-1, +1, 0, 0)
        glVertex4f(+1, +1, 0, 0)
        glVertex4f(+1, -1, 0, 0)
        glVertex4f(-1, -1, 0, 0)
        glEnd()
        
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, 1)
        glColor4f(0, 0, 0.5, 0.5)
        glVertex4f(0, 0, 0, 1)
        glVertex4f(-1, -1, 0, 0)
        glVertex4f(+1, -1, 0, 0)
        glVertex4f(+1, +1, 0, 0)
        glVertex4f(-1, +1, 0, 0)
        glVertex4f(-1, -1, 0, 0)
        glEnd()
        
        glDisable(GL_BLEND)
        
        glEnable(GL_LIGHTING)
        
        # underwater color
        if pos[2] > 0:
            glPushMatrix()
            glLoadIdentity()
            
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glMatrixMode(GL_MODELVIEW)
            
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glBegin(GL_QUADS)
            glColor4f(0, 0, 1, 0.5)
            glVertex3f(-1, -1, 0)
            glVertex3f(+1, -1, 0)
            glVertex3f(+1, +1, 0)
            glVertex3f(-1, +1, 0)
            glEnd()
            glDisable(GL_BLEND)
            
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
            
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            
            glPopMatrix()

if __name__ == '__main__':
    i = Interface()
    i.init()
    while True:
        i.step()
