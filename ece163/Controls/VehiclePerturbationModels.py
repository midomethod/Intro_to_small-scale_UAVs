import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations


"""
vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = math.radians(6.0)
    Kappastar = -1.0 / 150.0

    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if check:
        print("Optimization successful")
    else:
        print("Model converged outside of valid inputs, change parameters and try again")
"""

def CreateTransferFunction(trimState, trimInputs):
    """Returns Transfer Function from linearized TF class"""
    TF = Linearized.transferFunctions()

    # Straight from the states
    TF.Va_trim = trimState.Va
    TF.alpha_trim = trimState.alpha
    TF.beta_trim = trimState.beta
    TF.gamma_trim = trimState.pitch -trimState.alpha
    TF.theta_trim = trimState.pitch
    TF.phi_trim = trimState.roll

    throttle = trimInputs.Throttle
    elevator = trimInputs.Elevator

    # Roll TransferFn
    TF.a_phi1 = -0.25*VPC.rho*TF.Va_trim*VPC.S*(VPC.b**2)*VPC.Cpp
    TF.a_phi2 = 0.5*VPC.rho*(TF.Va_trim**2)*VPC.S*VPC.b*VPC.CpdeltaA

    # Sideslip TransferFn
    TF.a_beta1 = -0.5*VPC.rho*(TF.Va_trim)*VPC.S*VPC.CYbeta/VPC.mass
    TF.a_beta2 = 0.5*VPC.rho*(TF.Va_trim)*VPC.S*VPC.CYdeltaR/VPC.mass

    # Pitch TransferFn
    TF.a_theta1 = -0.25*VPC.rho*TF.Va_trim*(VPC.c**2)*VPC.S*VPC.CMq/VPC.Jyy
    TF.a_theta2 = -0.5*VPC.rho*(TF.Va_trim**2)*VPC.c*VPC.S*VPC.CMalpha/VPC.Jyy
    TF.a_theta3 = 0.5*VPC.rho*(TF.Va_trim**2)*VPC.c*VPC.S*VPC.CMdeltaE/VPC.Jyy

    # Airspeed TransferFn
        # Uses dthrust functions
    TF.a_V1 = (VPC.rho*TF.Va_trim*VPC.S*(VPC.CD0+VPC.CDalpha*TF.alpha_trim+VPC.CDdeltaE*elevator)-dThrust_dVa(TF.Va_trim,throttle))/VPC.mass
    TF.a_V2 = dThrust_dThrottle(TF.Va_trim,throttle)/VPC.mass
    TF.a_V3 = VPC.g0*math.cos(TF.theta_trim-TF.alpha_trim)

    return TF

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    """Calculate change in thrust wrt change in throttle"""
    # The small difference is epsilon = dDeltaT
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Thrust0, a = VAM.CalculatePropForces(Va,Throttle)
    Thrust1, a = VAM.CalculatePropForces(Va,Throttle+epsilon)

    # Return the derivative
    return (Thrust1-Thrust0)/epsilon

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    """Caulculate change in Thrust wrt va"""
    # The small difference is epsilon = dVa
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Thrust0, a = VAM.CalculatePropForces(Va,Throttle)
    Thrust1, a = VAM.CalculatePropForces(Va+epsilon,Throttle)

    # Return the derivative
    return (Thrust1-Thrust0)/epsilon