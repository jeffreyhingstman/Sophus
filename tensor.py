import numpy as np

class tensor():
    def __init__(self, H, low, upp):
        self.H = H
        self.low = low
        self.upp = upp

    def __mul__(self, other):
        return tensor(self.H @ other.H, other.low, self.upp)

    def info(self):
        print("Tensor from '{}' to '{}'".format(self.low, self.upp)) 

H1def = np.array([[1, 2],[3, 4]])
H2def = np.array([[5, 6],[7, 8]])

H1 = tensor(H1def, "i", "j")
H2 = tensor(H2def, "j", "k")

H3 = H2 * H1
H1.info()
H2.info()
H3.info()


