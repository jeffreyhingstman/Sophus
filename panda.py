from math import pi, sin, cos
import os
import sys
import SophusLibrary as Soph
import numpy as np

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import Filename
from scipy.spatial.transform import Rotation as Rot
from panda3d.core import Vec3 

mydir = os.path.abspath(sys.path[0])

# Convert that to panda's unix-style notation.
mydir = Filename.fromOsSpecific(mydir).getFullpath()
print("mdir: ", mydir)

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.pos_x = 0.0
        self.H_body = Soph.Rp2hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[1], [1], [1]])) 

        # Load and transform the panda actor.
        self.m = self.loader.loadModel("models/box.egg")
        self.m.setScale(4.0, 4.0, 4.0)
        self.m.reparentTo(self.render)

        # Load and transform the panda actor.
        self.b = self.loader.loadModel("models/box.egg")
        self.b.setScale(4.0, 4.0, 4.0)
        self.b.setPos(10, 1, 10)
        self.b.reparentTo(self.render)
        
        self.taskMgr.add(self.moveObj, "MoveObject")
        self.taskMgr.add(self.rigidBody, "RigidBody")

    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

    def moveObj(self, task):
        RotMat, p = Soph.hom2Rp(self.H_body)
        R = Rot.from_matrix(RotMat)
        Hpr = R.as_euler('zxy', degrees=True)
        #print("heading: {}, pitch = {}, roll = {}".format(Hpr[0], Hpr[1], Hpr[2]))
        self.m.setPos(p[0], p[1], p[2])
        self.m.setHpr(Hpr[0], Hpr[1], Hpr[2])
        return Task.cont 

    def rigidBody(self, task):   
        TIMESTEP = 0.1
        w_body = np.array([[0.000001], [0.1], [0.000001]])
        v_body = np.array([[0.1], [0.5], [0.3]])  
        expm = Soph.expm.se3_TO_SE3(w_body, v_body, TIMESTEP)
        self.H_body = np.dot(expm, self.H_body)
        return Task.cont



app = MyApp()
app.run()