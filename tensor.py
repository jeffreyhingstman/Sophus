import numpy as np

class tensor():
    def __init__(self, mat, low, upp):
        self.mat = mat
        self.low = low
        self.upp = upp

    def __mul__(self, other):
        if self.low != other.upp:
            print("Warning: Invld mapping, non-matching coordinate frames '{}' and '{}'".format(other.upp, self.low))
        return Hmatrix(self.mat @ other.mat, other.low, self.upp)

    def info(self):
        print("Homogeneous matrix from '{}' to '{}'".format(self.low, self.upp)) 

    def valid(self):
        # ToDo
        return 0

class Rmatrix(tensor):
    def __init__(self, R, low, upp):
        self.mat      = R
        self.low    = low
        self.upp    = upp  

class Hmatrix(tensor):
    def __init__(self, H, low, upp):
        self.mat      = H
        self.low    = low
        self.upp    = upp  

H1def = np.array([[1, 2],[3, 4]])
H2def = np.array([[5, 6],[7, 8]])

R1_def = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R2_def = np.array([[0, 1, 0],[-1, 0, 0],[0, 0, 1]])

R1 = Rmatrix(R1_def, 'i', 'j')
R2 = Rmatrix(R2_def, 'i', 'j')

R3 = R2 * R1
print("R3.R: ", R3.mat)

H1 = Hmatrix(H1def, "i", "v")
H2 = Hmatrix(H2def, "v", "k")

H3 = H2 * H1
H3.info()


