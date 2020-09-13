from math import pi, sin, cos
import os
import sys

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from panda3d.core import Filename

mydir = os.path.abspath(sys.path[0])

# Convert that to panda's unix-style notation.
mydir = Filename.fromOsSpecific(mydir).getFullpath()
print("mdir: ", mydir)

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.pos_x = 0.0

        # Load and transform the panda actor.
        self.m = self.loader.loadModel("models/box.egg")
        self.m.setScale(4.0, 4.0, 4.0)
        self.m.reparentTo(self.render)

        # Load and transform the panda actor.
        self.b = self.loader.loadModel("models/box.egg")
        self.b.setScale(2.0, 2.0, 2.0)
        self.b.setPos(10, 1, 10)
        self.b.reparentTo(self.render)
        
        self.taskMgr.add(self.moveObj, "MoveObject")

    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

    def moveObj(self, task):
        self.m.setPos(sin(task.time), 0, 0)
        return Task.cont 




app = MyApp()
app.run()