import numpy as np
import SophusLibrary as Lib
import Sophus as Soph

H_orig  = Lib.I4x4()

H_initA = Lib.rp2h(Lib.I3x3(), np.array([[0],[0],[0]]))
H_initB = Lib.rp2h(Lib.I3x3(), Lib.I3_1())

H1 = Lib.Hmatrix(H_initA, "O", "i")
H2 = Lib.Hmatrix(H_initB, "i", "j")

T1 = Lib.Revolute(np.array([[0],[0],[1]]), np.array([[0],[0],[1]]))
#T2 = Lib.Revolute(np.array([[0.0],[0],[1.0]]), np.array([[10],[0],[0]]))
T2 = Lib.Prismatic(np.array([[0],[0],[1]]))

#define the rigid bodies
Origin = Soph.Origin("Origin")
BodyA = Soph.RigidBody("A", H1, Origin, T1)
BodyB = Soph.RigidBody("B", H2, BodyA, T2)

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