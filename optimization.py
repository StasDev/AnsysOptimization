"""Optimization module"""

from math import pow
import numpy as np

from test_script import logging
import my_orm

def random_generator(min_value = -89, max_value = 90):
    """
    Random generator.
    int:min_value, int:max_value -> int:x
    """
    rng = np.random.default_rng()
    return rng.integers(low = min_value, high = max_value)

class ArrayAngles():
    """Data storage"""
    def __init__(self, number_of_layers = 2, bytes_per_layer = 6, min_angle = -89, max_angle = 90):
        self.number_of_layers = number_of_layers
        self.bytes_per_layer = bytes_per_layer
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step_angles = (self.max_angle - self.min_angle) / (pow(2, self.bytes_per_layer) - 1)
        self.data = np.zeros(self.bytes_per_layer * self.number_of_layers, dtype = bool)
        for i in range(len(self.data)):
            self.data[i] = random_generator(0, 2)
    
    def get_number_of_bytes(self):
        return len(self.data)
    
    def get_angle(self, index):
        """
        This function encode angle value from data.
        int:index -> float:x
        """
        try:
            indexFirstBit = self.bytes_per_layer * (index - 1)
            positionAngle = 0
            for i in range(indexFirstBit, indexFirstBit + self.bytes_per_layer):
                positionAngle += self.data[i] * pow(2, i - indexFirstBit)
            return self.min_angle + self.step_angles * positionAngle
        except ValueError:
            logging('Failing to access the layer')

    def set_approximate_angle(self, angle, index):
        """
        This function write to self.data the value of angle which you want to write on it
        int:angle, int:index -> None
        """
        positionAngle = np.uintc((angle - self.min_angle) / self.step_angles)
        indexFirstBit = self.bytes_per_layer * (index - 1)
        oneBit = np.uintc(1)
        for i in range(indexFirstBit, indexFirstBit + self.bytes_per_layer):
            self.data[i] = positionAngle & oneBit
            positionAngle = positionAngle >> 1
    
    def write_angles_to_file(self, path = r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\angles.txt'):
        """
        This function write the new values in angles.txt
        """
        f = open(path, 'r')
        for index in self.number_of_layers:
            f.write(str(self.get_angle(index)) + '\n')
        f.close()
    
    def read_safety_factor_from_ansys(self, path = r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\results.txt'):
        f = open(path, 'r')
        