import math
from typing import List

from . import MatrixMath


def dcm2Euler(DCM):
    """
    Returns the ψ(yaw), θ(pitch), Φ(roll) based on the
    contents of the rotation matrix.
    """

    yaw = math.atan2(DCM[0][1], DCM[0][0])  # Since top-middle and top-left element make tangent, we take arctan
    pitch = math.asin(-DCM[0][2])           # Since top-right element is simply -sin(pitch) we take arcsin
    roll = math.atan2(DCM[1][2], DCM[2][2]) # Since middle-right and bottom-right element make tangent, we take arctan

    return yaw, pitch, roll

def euler2DCM(yaw, pitch, roll):
    """
    Returns the composite rotation matrix based on the yaw, pitch and roll
    """

    # Construct the 3 rotation matrices for pitch, roll and yaw
    i2v1 = [[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    v12v2 = [[math.cos(pitch), 0, -math.sin(pitch)], [0, 1, 0], [math.sin(pitch), 0, math.cos(pitch)]]
    v22b = [[1, 0, 0], [0, math.cos(roll), math.sin(roll)], [0, -math.sin(roll), math.cos(roll)]]

    # Multiply the matrices to get the composite rotation matrix
    DCM = MatrixMath.matrixMultiply(v22b, MatrixMath.matrixMultiply(v12v2, i2v1))

    return DCM

def ned2enu(points):
    """
    Swaps the N<->E and convert Down to Up (flip the sign)
    """
    Rned2enu = [[0, 1, 0], [1, 0, 0], [0, 0, -1]] # The top row exracts the N as E, second extracts E as N, and bottom row extracts -D as U
    enuPoints = MatrixMath.matrixMultiply(points, Rned2enu) # Multiply the rotation matrix to the coordinates to get result

    return enuPoints