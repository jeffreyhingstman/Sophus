import numpy as np
import SophusLibrary as Soph
import visualization as Vis

H_initA = Soph.Rp2hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[1], [1], [1]])) 
H_initB = Soph.Rp2hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[3], [3], [3]])) 

#define the rigid bodies
newBodyA = Vis.newRigidBody("newBodyA", H_initA)
newBodyB = Vis.newRigidBody("newBodyB", H_initB)

listOfBodies = []
n = 1
while n < 30:
    x = Vis.newRigidBody("obj_{}".format(n), H_initA * n)
    n += 1

# instantiate the simulation and run it
sim = Vis.RigidBodySimulation()
try: sim.run()
except SystemExit: print("Closed simulation")

# do some plotting of angles
#plt.plot( np.arange(len(app.H_arr)), app.H_arr)
#plt.plot( np.arange(len(app.p_arr)), app.p_arr)
#plt.plot( np.arange(len(app.r_arr)), app.r_arr)
#print("plotting")
#plt.show()
#print("closed plot")