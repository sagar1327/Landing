#!/bin/python3
import numpy as np


class EstimateHeight:
    """Calculates height of the UAV"""
    def __init__(self, edge_pixels, focal_length, length_of_square_plate, height_of_the_light_source,
                 length_of_the_sq_plate_arm, dist_of_the_edge_from_deck):
        self.p = 3*10**(-6)
        self.Is = self.p*edge_pixels
        self.L = length_of_square_plate
        self.f = focal_length
        self.C = (np.sqrt(2)*self.Is)/(self.L*self.f)
        self.hl = height_of_the_light_source
        self.lr = length_of_the_sq_plate_arm
        self.ds = dist_of_the_edge_from_deck

    def height(self):
        height = self.hl*((self.C*self.lr) - 1)/(1 - (self.C*self.ds))
        print("The current height of the UAV is {}.\n".format(height))


def main(edge_pixels, focal_length, length_of_square_plate, height_of_the_light_source,
         length_of_the_sq_plate_arm, dist_of_the_edge_from_deck):
    h = EstimateHeight(edge_pixels, focal_length, length_of_square_plate, height_of_the_light_source,
                       length_of_the_sq_plate_arm, dist_of_the_edge_from_deck)
    h.height()


if __name__ == "__main__":
    main(127, 0.00083, 0.083, 0.2,
         0.12, 0.18)
