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
        expm = Soph.expm.se3_TO_SE3(self.w_body, self.v_body, self.timestep)
        self.H_body =  self.H_body @ expm # update value of H_body w.r.t. previous time step

        self.H_abs = self.objH_base.H_abs @ self.H_body
        RotMat, p = Soph.Hom_TO_Rp(self.H_abs)
        R = Rot.from_matrix(RotMat)
        quat = R.as_quat()
        self.body.setQuat(Quat(quat[3], quat[0], quat[1], quat[2]))
        self.body.setPos(p[0], p[1], p[2])
        return Task.cont
        
    def returnEuler(self, task):
        return self.body.getHpr() 

class Origin(RigidBody):
    def __init__(self, name):
        self.name = name
        self.H_base = Soph.I4x4()
        self.H_abs = Soph.I4x4()
        self.H_body = Soph.I4x4()
        self.body = None   # leave empty, later to be filled in by task manager 
        Global_RigidBodies.append(self)