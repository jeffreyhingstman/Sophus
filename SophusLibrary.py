import numpy as np 
import scipy 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


#todo: add zeta and zeta_hat for twist coordinates and homogeneous representation


def I4x4(): 
    return np.eye(4)
def I3x3(): 
    return np.eye(3)
def I3_1(): 
    return np.array([[1, 1, 1]]).T

def vec3(x1, x2, x3):
    return np.array([[x1, x2, x3]]).T

def skew(vec):
    # returns the 3x3 skew-symmetric matrix of a vector 
    # use: skew(np.array([a, b, c])) 
    return np.array([   [0.0, -vec[2, 0], vec[1, 0]], 
                        [vec[2, 0], 0.0, -vec[0, 0]], 
                        [-vec[1, 0], vec[0, 0], 0.0]
                    ])

def coord_from_h(h_matrix):
    v0 = h_matrix[0, 0]
    v1 = h_matrix[0, 1]
    v2 = h_matrix[0, 2]
    w0 = -h_matrix[1, 2]
    w1 = h_matrix[0, 2]
    w2 = h_matrix[1, 0]
    return np.array([[v0, v1, v2, w0, w1, w2]]).T


def concat_2x2(ind_1_1, ind_1_2, ind_2_1, ind_2_2):
    ind_1 = np.concatenate((ind_1_1, ind_1_2), axis = 1)
    ind_2 = np.concatenate((ind_2_1, ind_2_2), axis = 1)
    return np.concatenate((ind_1, ind_2), axis = 0)

def Hom_TO_Rp(H):
    # homogeneous matrix to rotation (R) + translation point (p)=
    return np.take(H, [0, 1, 2], 1)[0:3], np.take(H, [3], 1)[0:3] 

def Rp_TO_Hom(R, p):
    # Rotation + translation point to homogeneous matrix
    return concat_2x2(R, p, np.zeros((1, 3)), np.identity(1))

def Rp_TO_HomInv(R, p):
    # return the 4x4 inverse homogeneous matrix of a rotation matrix R and translation vector p
    return concat_2x2(R.T, np.dot(-R.T, p), np.zeros((1, 3)), np.identity(1))

class expm:
    # class of exponential mapping methods 
    @classmethod
    def so3_TO_SO3(self, w_skew, theta):
        # returns the exponential map using Rodrigues' formula, Has a 3x3 matrix as input, mapping so3-->SO(3), i.e. group of rotations
        # the magnitude of w_skew is normalized with the 2-norm.
        norm_w = np.linalg.norm(w_skew, 2)
        return np.identity(3) + (w_skew / norm_w) * np.sin(norm_w * theta) + (np.dot((w_skew / norm_w), (w_skew / norm_w))) * (1 - np.cos(norm_w * theta))

    @classmethod
    def se3_TO_SE3(self, w, v, dT):
        # (exponential map) maps the special Euclidian algebra (or 'generator') se3 to the special Euclidian group SE3:
        # g: se3 --> SE3, i.e. velocity (twist) configuration space.
        # Returns a homogeneous matrix 
        norm_w = np.linalg.norm(w, 2)
        SO3 = expm.so3_TO_SO3(skew(w), dT)
        I = np.identity(3)
        w_skw_n = skew(w) / norm_w
        w_n = w / norm_w
        ind_1_1 = SO3
        ind_1_2 = np.dot((I - SO3), np.dot((w_skw_n), v)) + np.dot(np.dot(w_n, (np.dot(w_n.T, v))), dT)
        ind_2_1 = np.zeros((1, 3))
        ind_2_2 = np.array([[1]])
        ind_1 = np.concatenate((ind_1_1, ind_1_2), axis = 1)
        ind_2 = np.concatenate((ind_2_1, ind_2_2), axis = 1)
        return np.concatenate((ind_1, ind_2), axis = 0)

class Screw:
    # class of screw theory method functions 
    @classmethod
    def pitch(self, w, v):
        # return the screw pitch
        norm_w = np.linalg.norm(w, 2)
        if norm_w == 0.0: 
            return np.NaN
        else:
            return np.dot(w.T, v) / ((np.linalg.norm(w, 2))**2)

    @classmethod
    def axis(self, w, v, _lambda):
        # return the screw axis 
        norm_w = np.linalg.norm(w, 2)
        if norm_w == 0.0:
            return np.dot(skew(w), v) / (norm_w**2) + np.dot(_lambda, w)
        else:
            return  np.dot(_lambda, v)

    @classmethod
    def magnitude(self, w, v):
        if w == 0.0:
            return np.linalg.norm(v, 2)
        else:
            return np.linalg.norm(w, 2)
    
    @classmethod
    def Screw_To_Twist_infPitch(self, v):
        ind_1_1 = 0.0 * np.identity(3)
        ind_1_2 = v
        ind_1 = np.concatenate((ind_1_1, ind_1_2), axis = 1)
        ind_2 =  np.zeros((1, 4))
        return np.concatenate((ind_1, ind_2), axis = 0)

    @classmethod
    def Screw_To_Twist_finPitch(self, w, q, h):
        ind_1_1 = skew(w)
        ind_1_2 = np.dot(-skew(w), q) + np.dot(h, w) 
        print("np.dot(h, w): ", np.dot(h, w))
        ind_1 = np.concatenate((ind_1_1, ind_1_2), axis = 1)
        ind_2 =  np.zeros((1, 4))
        return np.concatenate((ind_1, ind_2), axis = 0)
    
    def BodyVel2Twist(self, BodyVel):
        #ToDo
        return 0


        

    

class adjoint:
    # class of mappings within se3
    @classmethod
    def ad(self, R, p):
        # returns the 6x6 adjoint transformation. Used to transform twists from one coordinate frame to another.
        # e.g. if g_ab represents a rigid body transformation SE(3), of a body with coordinate frame B w.r.t. a fixed spatial coordinate frame A,
        # then the adjoint transformation of body- to spatial velocities is defined as 
        # V_spatial = Ad_g_ab * V_body            ( Vs = Ad_g . Vb )
        #  
        # see also: ad_inv
        return concat_2x2(R, np.dot(skew(p), R), np.zeros((3,3)), R)

    @classmethod
    def ad_inv(self, R, p):
        return concat_2x2(R.T, np.dot(-R.T, skew(p)), np.zeros((3, 3)), R.T)
        


