import numpy as np
import SophusLibrary as Lib
import Sophus as Soph

H_orig  = Lib.I4x4()

H_initA = Lib.rp2h(Lib.I3x3(), np.array([[0.1],[0.1],[0.1]]))
H_initB = Lib.rp2h(Lib.I3x3(), Lib.I3_1())

H1 = Lib.Hmatrix(H_orig, "O", "i")
H2 = Lib.Hmatrix(H_initB, "i", "j")

T1 = Lib.Revolute(np.array([[1.0],[0],[0]]), np.array([[0.000000001],[0],[0]]))
T2 = Lib.Revolute(np.array([[1.0],[0],[0.0]]), np.array([[0.000000001],[0],[1]]))
P1 = Lib.Prismatic(np.array([[0.1],[0.5],[0]]))

#define the rigid bodies
Origin = Soph.Origin("Origin")
BodyA = Soph.RigidBody("A", H1, Origin, P1)
BodyB = Soph.RigidBody("B", H2, BodyA, T2)
BodyC = Soph.RigidBody("C", H2, BodyB, T2)
#BodyD = Soph.RigidBody("D", H2, BodyC, T2)
#BodyE = Soph.RigidBody("E", H2, BodyD, T2)
#BodyF = Soph.RigidBody("F", H2, BodyE, T2)
#BodyG = Soph.RigidBody("G", H2, BodyF, T2)

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