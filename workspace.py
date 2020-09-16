import numpy as np
import SophusLibrary as Lib
import Sophus as Soph

H_orig  = Lib.I4x4()

H_initA = Lib.rp2h(Lib.I3x3(), Lib.I3_1())
H_initB = Lib.rp2h(Lib.I3x3(), Lib.I3_1() * 4)

H1 = Lib.Hmatrix(H_initA, "O", "i")
H2 = Lib.Hmatrix(H_initB, "i", "j")

#define the rigid bodies
Origin = Soph.Origin("Origin")
BodyA = Soph.RigidBody("A", H1, Origin,   w = Lib.vec3(0.0000001, 0, 0) , v = Lib.vec3(3.0, 0.0, 0))
BodyB = Soph.RigidBody("B", H2, BodyA,    w = Lib.vec3(0, 0, 10) , v = Lib.vec3(2, 2, 0))

# instantiate the simulation and run it#
sim = Soph.RigidBodySimulation()
try: sim.run()
except SystemExit: print("Closed simulation")

# do some plotting of angles
#plt.plot( np.arange(len(app.H_arr)), app.H_arr)
#plt.plot( np.arange(len(app.p_arr)), app.p_arr)
#plt.plot( np.arange(len(app.r_arr)), app.r_arr)
#print("plotting")
#plt.show()
#print("closed plot")