from math import pi, sin, cos
import os
import sys
import SophusLibrary as Lib
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


class RigidBody():
    def __init__(self, name, H_init, objH_base, Twist): 
        self.name = name
        self.H_base = Lib.Hmatrix.default(self)
        self.objH_base = objH_base
        self.H_body = H_init
        self.Twist = Twist   
        self.timestep = 0.01
        self.body = None   # leave empty, later to be filled in by task manager 
        self.H_abs = Lib.Hmatrix.default(self)
        Global_RigidBodies.append(self)


    def solve(self, task):
        self.H_base = self.objH_base.H_abs
        # based on current calue of H_body, calculate the homogeneous coordinates in the next step 
        # via the 4x4 exponential map.
        self.H_body = Lib.expm.SE3_TO_SE3(self.H_body, Lib.adjoint.ad_h(self.H_base).mat @ self.Twist, self.timestep)
        
        #calculate the h-matrix as a product of its internal mapping and the base coordinat frame
        self.H_abs =     self.H_body #* self.H_base  # ToDo: implement with inverse hom. coordinates? mapping back to world observer?????

        RotMat, p = Lib.h2rp(self.H_abs.mat)
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
        self.H_base =  Lib.Hmatrix(Lib.I4x4(), "O", "O")
        self.H_abs = Lib.Hmatrix(Lib.I4x4(), "O", "O")
        self.H_body = Lib.Hmatrix(Lib.I4x4(), "O", "O")
        self.body = None   # leave empty, later to be filled in by task manager 
        Global_RigidBodies.append(self)