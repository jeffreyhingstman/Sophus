from math import pi, sin, cos
import os
import sys
import SophusLibrary as Soph
import numpy as np

from matplotlib import pyplot as plt
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import Filename
from scipy.spatial.transform import Rotation as Rot
from panda3d.core import Quat

mydir = os.path.abspath(sys.path[0])
mydir = Filename.fromOsSpecific(mydir).getFullpath()

Global_RigidBodies = []

class RigidBodySimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.pos_x = 0.0
        self.H_body = Soph.Rp2hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[1], [1], [1]])) 
        self.H_arr = []
        self.p_arr = []
        self.r_arr = []
        self.Timestep = 0.01
        self.newOrigin()    # instantiate a (0,0,0) coordinate frame by default

        # add the list of rigid body object to the list of tasks
        for rb in Global_RigidBodies:
            print("added body {}".format(rb.name))
            self.taskMgr.add(rb.solve, rb.name)
            rb.body = self.loader.loadModel("models/zup-axis.egg")
            rb.body.setScale(0.5, 0.5, 0.5)
            rb.body.reparentTo(self.render)

    def newOrigin(self):
        self.origin = self.loader.loadModel("models/zup-axis.egg")
        self.origin.setScale(1.0, 1.0, 1.0)
        self.origin.reparentTo(self.render)

class newRigidBody():
    def __init__(self, name, H_init):
        self.name = name
        self.H_body = H_init
        self.w_body = np.array([[1.0], [0.0], [0.0]])
        self.v_body = np.array([[1.0], [0.0], [0.0]])  
        self.timestep = 0.01
        self.body = None   # leave empty, later to be filled in by task manager 
        Global_RigidBodies.append(self)

    def solve(self, task):
        # based on current calue of H_body, calculate the homogeneous coordinates in the next step 
        # via the 4x4 exponential map.
        expm = Soph.expm.se3_TO_SE3(self.w_body, self.v_body, self.timestep)
        self.H_body = np.dot(expm, self.H_body)

        RotMat, p = Soph.hom2Rp(self.H_body)
        R = Rot.from_matrix(RotMat)
        quat = R.as_quat()
        self.body.setPos(p[0], p[1], p[2])
        self.body.setQuat(Quat(quat[0], quat[1], quat[2], quat[3]))
        #Hpr = self.m.getHpr() 
        return Task.cont
