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

Global_RigidBodies  = []
#Global_Twists       = []

class RigidBodySimulation(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.Timestep = 0.01

        # add the list of rigid body object to the list of tasks
        for rb in Global_RigidBodies:
            print("added body {}".format(rb.name))

            if rb.name != "Origin":
                self.taskMgr.add(rb.solve, rb.name)

            rb.body = self.loader.loadModel("models/zup-axis.egg")
            rb.body.setScale(0.5, 0.5, 0.5)
            rb.body.reparentTo(self.render)

class Twist:
    def __init__(self, w, v): 
        self.w = w  # angular velocity vector
        self.v = v  # linear velocity vector

    #def __mul__(source, dest):
    #    return np.dot(1, 1) 


class RigidBody():
    def __init__(self, name, H_init, objH_base, w, v): 
        self.name = name
        self.objH_base = objH_base
        self.H_body = H_init
        self.w_body = w   # angular part of the twist
        self.v_body = v   # linear part of the twist
        self.timestep = 0.01
        self.body = None   # leave empty, later to be filled in by task manager 
        self.H_abs = None
        Global_RigidBodies.append(self)


    def solve(self, task):
        # based on current calue of H_body, calculate the homogeneous coordinates in the next step 
        # via the 4x4 exponential map.
        self.H_base = self.objH_base.H_body
        expm = Soph.expm.se3_TO_SE3(self.w_body, self.v_body, self.timestep)
        self.H_body = np.dot(expm, self.H_body)

        self.H_abs = np.dot(self.H_body,  self.H_base)
        RotMat, p = Soph.Hom_TO_Rp(self.H_abs)
        R = Rot.from_matrix(RotMat)
        quat = R.as_quat()
        self.body.setPos(p[0], p[1], p[2])
        self.body.setQuat(Quat(quat[0], quat[1], quat[2], quat[3]))
        return Task.cont
        
    def returnEuler(self, task):
        return self.body.getHpr() 

class Origin(RigidBody):
    def __init__(self, name):
        self.name = name
        self.H_base = Soph.Rp_TO_Hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[0], [0], [0]]))
        self.H_body = Soph.Rp_TO_Hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[0], [0], [0]]))
        self.body = None   # leave empty, later to be filled in by task manager 
        Global_RigidBodies.append(self)