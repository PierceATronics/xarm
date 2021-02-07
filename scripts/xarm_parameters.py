import numpy as np


class xArmParameters:
    '''
    Define the constant parameters of the xarm
    '''
    def __init__(self):

        #Define the xarm length parameters
        self.L_23 = 97.5; self.L_34 = 97.5; self.L_4E = 170.4;

        #home (zero) position of the arm
        self.M = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, self.L_23 + self.L_34 + self.L_4E],
                           [0, 0, 0, 1]])

        #Screw axis representation of each joints configuration.
        self.S1 = np.array([0, 0, 1, 0, 0, 0])
        self.S2 = np.array([0, 1, 0, 0, 0, 0])
        self.S3 = np.array([0, 1, 0, -1*self.L_23, 0, 0])
        self.S4 = np.array([0, 1, 0, -1*(self.L_23 + self.L_34), 0, 0])
        self.S5 = np.array([0, 0, 1, 0, 0, 0])

        self.Slist = np.array([self.S1, self.S2, self.S3, self.S4, self.S5]).T

if __name__ == "__main__":

    print(xArmParameters().M)
