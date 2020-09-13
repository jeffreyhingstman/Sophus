import numpy as np
import SophusLibrary as Soph # robotics toolbox 


w = np.array([[1, 1, 1]]).T # column vector 
w_skew = Soph.skew(w)
v = np.array([[1, 0, 0]]).T
theta = 3.141592 

expm = Soph.expm.so3_TO_SO3(Soph.skew(w), theta)
#print("se3: ", Expm.se3(w, v, theta))
#print("pitch h: ", Screw.pitch(w, v))
#print("axis l: ", Screw.axis(w, v, 2))



#print("twist from screw: ", Soph.coord_from_h(Soph.Screw.twistFromScrew_finPitch( w = np.array([[1, 1, 1]]).T, 
#                                                                        q = np.array([[1, 1, 1]]).T, 
#                                                                        h =  1.0)))


#print("adjoint: ", Soph.adjoint.ad(Soph.skew(w), w))
#print("adjoint_inverse: ", Soph.adjoint.ad_inv(Soph.skew(w), w))

#print("adjoint identity: ", np.dot(Soph.adjoint.ad(np.identity(3), w), Soph.adjoint.ad_inv(np.identity(3), w)))

#H_obj = np.array([[1, 0, 0, 2], [0, 3, 0, 4], [0, 0, 5, 6], [0, 0, 0, 1]])
#print("coords: ", Soph.hom2Rp(H_obj))
TIMESTEP = 0.01
N = 1000
n = 0

H_body = Soph.Rp2hom(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([[1], [1], [1]])) 
w_body = np.array([[0.1], [0.2], [0.3]])
v_body = np.array([[0.1], [0.2], [0.3]]) 

while n < N:

    expm = Soph.expm.se3_TO_SE3(w_body, v_body, TIMESTEP)

    H_body = np.dot(expm, H_body)
    print("H_body after: ", H_body)
    n += 1