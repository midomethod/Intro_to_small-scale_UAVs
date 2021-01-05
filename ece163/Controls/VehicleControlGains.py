import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    # Lateral and longitudinal gains are calculated here
    # No check is made for frequency separation
    
    # Initialize the control gains container
    CG = Controls.controlGains()

    # ROLL
    CG.kp_roll = tuningParameters.Wn_roll**2/linearizedModel.a_phi2
    CG.kd_roll = (2*tuningParameters.Zeta_roll*tuningParameters.Wn_roll-linearizedModel.a_phi1)/linearizedModel.a_phi2
    CG.ki_roll = 0.001

    # COURSE
    CG.ki_course = tuningParameters.Wn_course**2*linearizedModel.Va_trim/VPC.g0
    CG.kp_course = 2*tuningParameters.Wn_course*tuningParameters.Zeta_course*linearizedModel.Va_trim/VPC.g0

    # SIDESLIP
    CG.ki_sideslip = tuningParameters.Wn_sideslip**2/linearizedModel.a_beta2
    CG.kp_sideslip = (2*tuningParameters.Wn_sideslip*tuningParameters.Zeta_sideslip-linearizedModel.a_beta1)/linearizedModel.a_beta2

    # PITCH
    CG.kp_pitch = (tuningParameters.Wn_pitch**2-linearizedModel.a_theta2)/linearizedModel.a_theta3
    CG.kd_pitch = (2*tuningParameters.Wn_pitch*tuningParameters.Zeta_pitch-linearizedModel.a_theta1)/linearizedModel.a_theta3

    K_DC = CG.kp_pitch*linearizedModel.a_theta3/(linearizedModel.a_theta2+CG.kp_pitch*linearizedModel.a_theta3) 

    # ALTITUDE
    CG.ki_altitude = tuningParameters.Wn_altitude**2/(K_DC*linearizedModel.Va_trim)
    CG.kp_altitude = 2*tuningParameters.Wn_altitude*tuningParameters.Zeta_altitude/(K_DC*linearizedModel.Va_trim)

    # V from pitch
    CG.ki_SpeedfromElevator = -tuningParameters.Wn_SpeedfromElevator**2/(K_DC*VPC.g0)
    CG.kp_SpeedfromElevator = (linearizedModel.a_V1-2*tuningParameters.Wn_SpeedfromElevator*tuningParameters.Zeta_SpeedfromElevator)/(K_DC*VPC.g0)

    # V from throttle
    CG.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle**2/linearizedModel.a_V2
    CG.kp_SpeedfromThrottle = (2*tuningParameters.Zeta_SpeedfromThrottle*tuningParameters.Wn_SpeedfromThrottle-linearizedModel.a_V1)/linearizedModel.a_V2

    return CG

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    # Compute tuning parameters based on control gains.
    # Utilizes a try block to catch taking a square root of a negative number
    # Returns an empty init control tuning if exception is detected.

    try:
        # Trying to compute the value normally,  including the sqrt of negative number
        
        # Initialize the tuning parameters container
        TP = Controls.controlTuning()

        # ROLL
        TP.Wn_roll = math.sqrt(controlGains.kp_roll*linearizedModel.a_phi2)
        TP.Zeta_roll = 0.5*(linearizedModel.a_phi1+linearizedModel.a_phi2*controlGains.kd_roll)/TP.Wn_roll

        # COURSE
        TP.Wn_course = math.sqrt(VPC.g0*controlGains.ki_course/linearizedModel.Va_trim)
        TP.Zeta_course = 0.5*VPC.g0*controlGains.kp_course/(linearizedModel.Va_trim*TP.Wn_course)

        # SIDESLIP
        TP.Wn_sideslip = math.sqrt(linearizedModel.a_beta2*controlGains.ki_sideslip)
        TP.Zeta_sideslip = 0.5*(linearizedModel.a_beta1+linearizedModel.a_beta2*controlGains.kp_sideslip)/TP.Wn_sideslip

        # PITCH
        TP.Wn_pitch = math.sqrt(linearizedModel.a_theta2+linearizedModel.a_theta3*controlGains.kp_pitch)
        TP.Zeta_pitch = 0.5*(linearizedModel.a_theta1+controlGains.kd_pitch*linearizedModel.a_theta3)/TP.Wn_pitch

        K_DC = controlGains.kp_pitch*linearizedModel.a_theta3/(linearizedModel.a_theta2+controlGains.kp_pitch*linearizedModel.a_theta3)

        # ALTITUDE
        TP.Wn_altitude = math.sqrt(K_DC*linearizedModel.Va_trim*controlGains.ki_altitude)
        TP.Zeta_altitude = 0.5*K_DC*linearizedModel.Va_trim*controlGains.kp_altitude/TP.Wn_altitude

        # V from pitch
        TP.Wn_SpeedfromElevator = math.sqrt(-controlGains.ki_SpeedfromElevator*VPC.g0*K_DC)
        TP.Zeta_SpeedfromElevator = 0.5*(linearizedModel.a_V1-K_DC*VPC.g0*controlGains.kp_SpeedfromElevator)/TP.Wn_SpeedfromElevator

        # V from throttle
        TP.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2*controlGains.ki_SpeedfromThrottle)
        TP.Zeta_SpeedfromThrottle = 0.5*(linearizedModel.a_V1+linearizedModel.a_V2*controlGains.kp_SpeedfromThrottle)/TP.Wn_SpeedfromThrottle

    except ValueError:
        # If a sqrt of negative number is attempted, return empty init'ed container
        TP = Controls.controlTuning()

    return TP

