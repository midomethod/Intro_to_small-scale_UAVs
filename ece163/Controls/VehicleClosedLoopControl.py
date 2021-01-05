import math
import sys
import pickle
import enum
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule

# This is the class for PDController
class PDControl():
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return
    
    # Implements the PD about the trim
    def Update(self,command=0.0, current=0.0, derivative=0.0):
        err = command-current
        Up = self.kp*err
        Ud = self.kd*(-derivative)
        U=Up+Ud+self.trim
        
        if U>=self.lowLimit and U<=self.highLimit:
            return U
        elif U>self.highLimit:
            return self.highLimit
        else:
            return self.lowLimit

    # Sets the gain values as well as the trim
    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp=kp
        self.kd=kd
        self.trim=trim
        self.lowLimit=lowLimit
        self.highLimit=highLimit
        return

# Implements the PI controller
class PIControl():
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT=dT
        self.kp=kp
        self.ki=ki
        self.trim=trim
        self.lowLimit=lowLimit
        self.highLimit=highLimit
        self.prevError=0
        self.accumulator = 0
        return

    # Implements the PI controller using the internal gains, about the trim
    def Update(self, command=0.0, current=0.0):
        err = command-current

        self.accumulator += 0.5*self.dT*(err+self.prevError)
        Up = self.kp*err
        Ui = self.ki*self.accumulator
        U = Up+Ui+self.trim

        if U>=self.lowLimit and U<=self.highLimit:
            self.prevError = err
            return U
        else:
            self.accumulator -= 0.5*self.dT*(err+self.prevError)
            self.prevError = err
            
            if U>self.highLimit:
                return self.highLimit
            else:
                return self.lowLimit

    # Resets the accumulator as well as errorPrev
    def resetIntegrator(self):
        self.accumulator = 0
        self.prevError = 0
        return

    # Sets gains
    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT=dT
        self.kp=kp
        self.ki=ki
        self.trim=trim
        self.lowLimit=lowLimit
        self.highLimit=highLimit
        return

# PID controller that'll be used in the successive loop closure
class PIDControl():
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT=dT
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.trim=trim
        self.lowLimit=lowLimit
        self.highLimit=highLimit
        self.prevError = 0
        self.accumulator = 0
        return

    def __repr__(self):
        return "PIDControl(dT={},kp={},kd={},ki={},trim={},lowLimit={},highLimit={})".format(self.dT,self.kp,self.kd,self.ki,self.trim,self.lowLimit,self.highLimit)

    # Finds the output based on command, current and the derivative using trapezoidal integration
    def Update(self, command=0.0, current=0.0, derivative=0.0):
        err = command-current

        self.accumulator += 0.5*self.dT*(err+self.prevError)
        Up = self.kp*err
        Ui = self.ki*self.accumulator
        Ud = self.kd*(-derivative)
        U = Up+Ui+Ud+self.trim

        if U>=self.lowLimit and U<=self.highLimit:
            self.prevError = err
            return U
        else:
            self.accumulator -= 0.5*self.dT*(err+self.prevError)
            self.prevError = err
            
            if U>self.highLimit:
                return self.highLimit
            else:
                return self.lowLimit
        return
    
    # Reset integrator and the previous error
    def resetIntegrator(self):
        self.accumulator = 0
        self.prevError = 0
        return

    # Set gains
    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0): 
        self.dT=dT
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.trim=trim
        self.lowLimit=lowLimit
        self.highLimit=highLimit
        return

# The class to implement the actual loop closure
class VehicleClosedLoopControl():
    def __init__(self, dT=0.01):
        self.dT=dT
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.CG = Controls.controlGains()
        self.TI = Inputs.controlInputs()
        self.CS = Inputs.controlInputs()
        self.aileronFromPhi = PIDControl()
        self.rudderFromBeta = PIControl()
        self.PhiFromChi = PIControl()
        self.elevatorFromTheta = PDControl()
        self.ThetaFromH = PIControl()
        self.ThetaFromVa = PIControl()
        self.ThrottleFromVa = PIControl()
        self.climbMode = Controls.AltitudeStates.HOLDING

        self.Lateral1=0
        self.Lateral2=0

        return

    # calculates the control surfaces and updates the VAM based on the command course and airspeed and height
    def Update(self, referenceCommands=Controls.referenceCommands()):
        # Adjust the internal course by ±2π if deltaChi is too large
        deltaCHI = referenceCommands.commandedCourse - self.VAM.vehicleDynamics.state.chi
        if deltaCHI >= math.pi:
            self.VAM.vehicleDynamics.state.chi += 2 * math.pi
        elif deltaCHI <= math.pi:
            self.VAM.vehicleDynamics.state.chi -= 2 * math.pi

        # Get roll command from course
        Roll_c = self.PhiFromChi.Update(referenceCommands.commandedCourse, self.VAM.vehicleDynamics.state.chi)

        # Get aileron command from roll
        Aileron_c = self.aileronFromPhi.Update(Roll_c, self.VAM.vehicleDynamics.state.roll, self.VAM.vehicleDynamics.state.p)

        # Get rudder command from sideslip
        Rudder_c = self.rudderFromBeta.Update(0, self.VAM.vehicleDynamics.state.beta)

        # Determine the altitude hold status {CLIMBING, DESCENDING, HOLDING}
        altitude_current = -self.VAM.vehicleDynamics.state.pd

        # This is the altitude state machine used to change the climb mode and calculate throttle and pitch accordingly
        if altitude_current > referenceCommands.commandedAltitude + VPC.altitudeHoldZone:
            # The climb mode should be descending
            if self.climbMode is not Controls.AltitudeStates.DESCENDING:
                # Set climb mode if not already descending
                self.climbMode = Controls.AltitudeStates.DESCENDING
                # Reset integrator on the transition
                self.ThetaFromVa.resetIntegrator()
            # Turn off throttle if we should be descending
            Throttle_c = VPC.minControls.Throttle
            # Calculate pitch from airspeed
            Pitch_c = self.ThetaFromVa.Update(referenceCommands.commandedAirspeed, self.VAM.vehicleDynamics.state.Va)
        elif altitude_current < (referenceCommands.commandedAltitude - VPC.altitudeHoldZone):
            # The climb mode should be climbing
            if self.climbMode is not Controls.AltitudeStates.CLIMBING:
                # Set to climbing if not already climbing
                self.climbMode = Controls.AltitudeStates.CLIMBING
                self.ThetaFromVa.resetIntegrator()
            # Full throttle if we must climb
            Throttle_c = VPC.minControls.Throttle
            # Calculate pitch from airspeed if climbing            
            Pitch_c = self.ThetaFromVa.Update(referenceCommands.commandedAirspeed, self.VAM.vehicleDynamics.state.Va)
        else:
            # Otherwise we should be trying to hold altitude
            if(self.climbMode is not Controls.AltitudeStates.HOlDING):
                # Set mode to holding if not already so
                self.climbMode = Controls.AltitudeStates.HOLDING
                self.ThetaFromVa.resetIntegrator()
            # Throttle calculated with Va if holding
            Throttle_c = self.ThrottleFromVa.Update(referenceCommands.commandedAirspeed,
                                                    self.VAM.vehicleDynamics.state.Va)
            # Pitch should be calculated from altitude
            Pitch_c = self.ThetaFromH.Update(referenceCommands.commandedAltitude, altitude_current)

        # The command elevator is then calculated from pitch and q
        Elevator_c = self.elevatorFromTheta.Update(Pitch_c,
                                                self.VAM.vehicleDynamics.state.pitch,
                                                self.VAM.vehicleDynamics.state.q)

        # Set control surfaces
        self.CS.Throttle = Throttle_c
        self.CS.Elevator = Elevator_c
        self.CS.Rudder = Rudder_c
        self.CS.Aileron = Aileron_c

        # Alter the reference commands
        referenceCommands.commandedPitch = Pitch_c
        referenceCommands.commandedRoll = Roll_c

        # Update the VAM using the new control surfaces
        self.VAM.Update(self.CS)
        return

    def getControlGains(self):
        return self.CG
    
    def generateLateral(self, trimState = States.vehicleState(), trimInputs = Inputs.controlInputs()):
        
        # mode = True (beta implementation), mode = False (v implementation)
        beta_mode = True

        # Generates the matrices needed for the state-space control using equation 5.44 from textbook
        Y_v = (VPC.rho*VPC.S*VPC.b*trimState.v/(4*VPC.mass*trimState.Va))*(VPC.CYp*trimState.p+VPC.CYr*trimState.r)\
            + (VPC.rho*VPC.S*trimState.v/(VPC.mass))*(VPC.CY0+VPC.CYbeta*trimState.beta+VPC.CYdeltaA*trimInputs.aileron+VPC.CYdeltaR*trimInputs.rudder)\
            + (VPC.rho*VPC.S*VPC.CYbeta/(2*VPC.mass))*math.hypot(trimState.u,trimState.w)

        Y_p = trimState.w\
            + (VPC.rho*VPC.S*VPC.b*trimState.Va/(4*VPC.mass))*VPC.CYp

        Y_r = -trimState.u\
            + (VPC.rho*VPC.S*VPC.b*trimState.Va/(4*VPC.mass))*VPC.CYr

        Y_da = (VPC.rho*VPC.S*trimState.Va**2/(2*VPC.mass))*VPC.CYdeltaA

        Y_dr = (VPC.rho*VPC.S*trimState.Va**2/(2*VPC.mass))*VPC.CYdeltaR

        L_v = (VPC.rho*VPC.S*(VPC.b**2)*trimState.v/(4*trimState.Va))*(VPC.Cpp*trimState.p+VPC.Cpr*trimState.r)\
            + (VPC.rho*VPC.S*VPC.b*trimState.v)*(VPC.Cp0+VPC.Cpbeta*trimState.beta+VPC.CpdeltaA*trimInputs.aileron+VPC.CpdeltaR*trimInputs.rudder)\
            + (VPC.rho*VPC.S*VPC.b*VPC.Cpbeta/2)*math.hypot(trimState.u,trimState.w)

        L_p = VPC.gamma1*trimState.q\
            + (VPC.rho*VPC.S*(VPC.b**2)*trimState.Va/4)*VPC.Cpp

        L_r = VPC.gamma2*trimState.q\
            + (VPC.rho*VPC.S*(VPC.b**2)*trimState.Va/4)*VPC.Cpr

        L_da = (VPC.rho*VPC.S*VPC.b*trimState.Va**2/2)*VPC.CpdeltaA

        L_dr = (VPC.rho*VPC.S*VPC.b*trimState.Va**2/2)*VPC.CpdeltaR

        N_v = (VPC.rho*VPC.S*(VPC.b**2)*trimState.v/(4*trimState.Va))*(VPC.Crp*trimState.p+VPC.Crr*trimState.r)\
            + (VPC.rho*VPC.S*VPC.b*trimState.v)*(VPC.Cr0+VPC.Crbeta*trimState.beta+VPC.CrdeltaA*trimInputs.aileron+VPC.CrdeltaR*trimInputs.rudder)\
            + (VPC.rho*VPC.S*VPC.b*VPC.Crbeta/2)*math.hypot(trimState.u,trimState.w)

        N_p = VPC.gamma7*trimState.q\
            + (VPC.rho*VPC.S*(VPC.b**2)*trimState.Va/4)*VPC.Crp

        N_r = -VPC.gamma1*trimState.q\
            + (VPC.rho*VPC.S*(VPC.b**2)*trimState.Va/4)*VPC.Crr

        N_da = (VPC.rho*VPC.S*VPC.b*trimState.Va**2/2)*VPC.CrdeltaA

        N_dr = (VPC.rho*VPC.S*VPC.b*trimState.Va**2/2)*VPC.CrdeltaR

        # Construct the matrices (beta version) using the equation 5.44
        if beta_mode:
            Mat1 = [[Y_v, Y_p/(trimState.Va*math.cos(trimState.beta)), Y_r/(trimState.Va*math.cos(trimState.beta)),\
                        VPC.g0*math.cos(trimState.pitch)*math.cos(trimState.roll)/(trimState.Va*math.cos(trimState.beta)), 0],\
                    [L_v*trimState.Va*math.cos(trimState.beta), L_p, L_r, 0, 0],\
                    [N_v*trimState.Va*math.cos(trimState.beta), N_p, N_r, 0, 0],\
                    [0, 1, math.cos(trimState.roll)*math.tan(trimState.pitch),\
                        trimState.q*math.cos(trimState.roll)*math.tan(trimState.pitch)-trimState.r*math.sin(trimState.roll)*math.tan(trimState.pitch),0],\
                    [0, 0, math.cos(trimState.roll)/math.cos(trimState.pitch),\
                        trimState.p*math.cos(trimState.roll)/math.cos(trimState.pitch)-trimState.r*math.sin(trimState.roll)/math.cos(trimState.pitch),0]]
            
            Mat2 = [[Y_da/(trimState.Va*math.cos(trimState.beta)),Y_dr/(trimState.Va*math.cos(trimState.beta))],\
                    [L_da, L_dr],\
                    [N_da, N_dr],\
                    [0, 0],\
                    [0, 0]]
        # (v version in case it comes up)            
        else:
            Mat1 = [[Y_v, Y_p, Y_r, VPC.g0*math.cos(trimState.pitch)*math.cos(trimState.roll), 0],\
                    [L_v*trimState.Va*math.cos(trimState.beta), L_p, L_r, 0, 0],\
                    [N_v*trimState.Va*math.cos(trimState.beta), N_p, N_r, 0, 0],\
                    [0, 1, math.cos(trimState.roll)*math.tan(trimState.pitch),\
                        trimState.q*math.cos(trimState.roll)*math.tan(trimState.pitch)-trimState.r*math.sin(trimState.roll)*math.tan(trimState.pitch),0],\
                    [0, 0, math.cos(trimState.roll)/math.cos(trimState.pitch),\
                        trimState.p*math.cos(trimState.roll)/math.cos(trimState.pitch)-trimState.r*math.sin(trimState.roll)/math.cos(trimState.pitch),0]]
            
            Mat2 = [[Y_da,Y_dr],\
                    [L_da, L_dr],\
                    [N_da, N_dr],\
                    [0, 0],\
                    [0, 0]]

        self.Lateral1 = Mat1
        self.Lateral2 = Mat2
        
        return Mat1, Mat2

    def getVehicleAerodynamicsModel(self):
        return self.VAM

    def getVehicleControlSurfaces(self):
        return self.CS

    def getVehicleState(self):
        return self.VAM.vehicleDynamics.state

    # Resets the model as well as all of the integrators
    def reset(self):
        self.VAM.reset()
        self.aileronFromPhi.resetIntegrator()
        self.rudderFromBeta.resetIntegrator()
        self.PhiFromChi.resetIntegrator()
        self.ThetaFromH.resetIntegrator()
        self.ThetaFromVa.resetIntegrator()
        self.ThrottleFromVa.resetIntegrator()
        return
    
    # Set internal gains. To be used from outside the class
    def setControlGains(self, CG):
        self.CG = CG
        self.aileronFromPhi.setPIDGains(self.VAM.vehicleDynamics.dT, self.CG.kp_roll, self.CG.kd_roll,self.CG.ki_roll,self.TI.Aileron,VPC.minControls.Aileron,VPC.maxControls.Aileron)
        self.PhiFromChi.setPIGains(self.VAM.vehicleDynamics.dT,self.CG.kp_course,self.CG.ki_course,0.0,-math.radians(VPC.bankAngleLimit),math.radians(VPC.bankAngleLimit))
        self.rudderFromBeta.setPIGains(self.VAM.vehicleDynamics.dT,self.CG.kp_sideslip,self.CG.ki_sideslip,self.TI.Rudder,VPC.minControls.Rudder,VPC.maxControls.Rudder)
        self.elevatorFromTheta.setPDGains(self.CG.kp_pitch,self.CG.kd_pitch,self.TI.Elevator,VPC.minControls.Elevator,VPC.maxControls.Elevator)
        self.ThrottleFromVa.setPIGains(self.VAM.vehicleDynamics.dT,self.CG.kp_SpeedfromThrottle,self.CG.ki_SpeedfromThrottle,self.TI.Throttle,VPC.minControls.Throttle,VPC.maxControls.Throttle)
        self.ThetaFromH.setPIGains(self.VAM.vehicleDynamics.dT,self.CG.kp_altitude,self.CG.ki_altitude,0.0,-math.radians(VPC.pitchAngleLimit),math.radians(VPC.pitchAngleLimit))
        self.ThetaFromVa.setPIGains(self.VAM.vehicleDynamics.dT,self.CG.kp_SpeedfromElevator,self.CG.ki_SpeedfromElevator,0.0,-math.radians(VPC.pitchAngleLimit),math.radians(VPC.pitchAngleLimit))
        return
    
    def getTrimInputs(self):
        return self.TI

    def setTrimInputs(self, TI=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.TI = TI
        return

    def setVehicleState(self, state): 
        self.VAM.vehicleDynamics.state = state  
        return
    
