import numpy as np
import SophusLibrary as Lib
import Sophus as Soph

H_orig  = Lib.I4x4()

H_initA = Lib.rp2h(Lib.I3x3(), Lib.vec3(-5.0, -5.0, 0))
H_initB = Lib.rp2h(Lib.I3x3(), Lib.vec3(-5, -5, 0))

H1 = Lib.Hmatrix(H_initA, "O", "i")
H2 = Lib.Hmatrix(H_initB, "i", "j")

T1 = Lib.Revolute(np.array([[0.0],[0],[1.0]]), np.array([[5.0],[5.0],[0]]))
T2 = Lib.Revolute(np.array([[0.0],[0],[1.0]]), np.array([[5.0],[5.0],[0]]))

#P1 = Lib.Prismatic(np.array([[0.00001],[0.000005],[0]]))

#define the rigid bodies
Origin = Soph.Origin("Origin")
BodyA = Soph.RigidBody("A", H1, Origin, T1)
BodyB = Soph.RigidBody("B", H2, BodyA, T2)
# instantiate the simulation and run it#
sim = Soph.RigidBodySimulation()
try: sim.run()
except SystemExit: print("Closed simulation")
