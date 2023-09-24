import numpy as np
import cosimpy

class CustomInductor:
    def __init__(self, L_value="10uH", R_value=5):
        self.L = float(L_value[:-2]) * 1e-6
        self.R = R_value

    def simulate(self, frequencies):
        nPoints = [20, 20, 20]
        b_field = np.zeros((1, 1, 3, np.prod(nPoints)))
        b_field[:, :, 1, :] = 0.1e-6

        s_coil = cosimpy.S_Matrix.sMatrixRLseries(self.R, self.L, frequencies)
        em_coil = cosimpy.EM_Field([128e6], nPoints, b_field)

        rf_coil = cosimpy.RF_Coil(s_coil, em_coil)
        return rf_coil

def calculate_rotation_from_field(b_field):
    # Simplified assumption: rotation is directly proportional to the magnetic field's magnitude.
    rotation_angle = np.sum(np.abs(b_field)) * 1e4
    return rotation_angle

def main():
    # Initialize the inductors using the CustomInductor class
    north = CustomInductor("10uH")
    south = CustomInductor("10uH")
    east = CustomInductor("10uH")
    west = CustomInductor("10uH")

    frequencies = np.linspace(50e6, 250e6, 1001)

    rf_coil_north = north.simulate(frequencies)
    rf_coil_south = south.simulate(frequencies)
    rf_coil_east = east.simulate(frequencies)
    rf_coil_west = west.simulate(frequencies)

    total_rotation = 0
    for coil in [rf_coil_north, rf_coil_south, rf_coil_east, rf_coil_west]:
        total_rotation += calculate_rotation_from_field(coil.em_field.b_field)

    print(f"Total rotation induced in the metal ball: {total_rotation} degrees")

if __name__ == "__main__":
    main()
