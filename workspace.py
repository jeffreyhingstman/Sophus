import numpy as np
import SophusLibrary as Lib
import Sophus as Soph

H_orig  = Lib.I4x4()

H_initA = Lib.rp2h(Lib.I3x3(), Lib.vec3(5, 0, 0))
H_initB = Lib.rp2h(Lib.I3x3(), Lib.vec3(10, 0, 0))

H1 = Lib.Hmatrix(H_initA, "O", "i")
H2 = Lib.Hmatrix(H_initB, "i", "j")

T1 = Lib.Revolute(np.array([[0.0],[0],[1.0]]), np.array([[-5.0],[0.0],[0]]), low='O', upp='i')
T2 = Lib.Revolute(np.array([[0.0],[0],[2.0]]), np.array([[-10.0],[0.0],[0]]), low='i', upp='j')

#P1 = Lib.Prismatic(np.array([[0.00001],[0.000005],[0]]))

#define the rigid bodies
Origin = Soph.Origin("Origin")
BodyA = Soph.RigidBody("A", H1, Origin, T1)
BodyB = Soph.RigidBody("B", H2, BodyA, T2)
BodyC = Soph.RigidBody("B", H2, BodyB, T2)
BodyD = Soph.RigidBody("B", H2, BodyC, T2)
BodyE = Soph.RigidBody("B", H2, BodyD, T2)
BodyF = Soph.RigidBody("B", H2, BodyE, T2)
BodyG = Soph.RigidBody("B", H2, BodyF, T2)
BodyH = Soph.RigidBody("B", H2, BodyG, T2)
BodyI = Soph.RigidBody("B", H2, BodyH, T2)
BodyJ = Soph.RigidBody("B", H2, BodyI, T2)
BodyK = Soph.RigidBody("B", H2, BodyJ, T2)

# instantiate the simulation and run it#
sim = Soph.RigidBodySimulation()
try: sim.run()
except SystemExit: print("Closed simulation")
