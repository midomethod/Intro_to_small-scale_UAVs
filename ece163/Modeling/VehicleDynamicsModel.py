import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel():

    def __init__(self, dT=VPC.dT):
        self.state = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
        self.dT = dT
        self.dot = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)

        return

    def ForwardEuler(self, forcesMoments):
        # Find the derivative to be used in Integrate()
        dot = self.derivative(self.state,forcesMoments)

        # Integrate
        state=self.IntegrateState(self.dT,self.state,dot)

        return state

    def IntegrateState(self, dT, state, dot):
        # Use matrix exponential to propagate attitude
        newRot = MatrixMath.matrixMultiply(self.Rexp(dT,state,dot),state.R)

        # Extract attitude tuple from DCM
        Attituple = Rotations.dcm2Euler(newRot)
        newYaw = Attituple[0]
        newPitch = Attituple[1]
        newRoll = Attituple[2]

        # Perform simple forward integration on all other parameters
        newPn = state.pn+dT*dot.pn
        newPe = state.pe+dT*dot.pe
        newPd = state.pd+dT*dot.pd
        newU = state.u+dT*dot.u
        newV = state.v+dT*dot.v
        newW = state.w+dT*dot.w
        newP = state.p+dT*dot.p
        newQ = state.q+dT*dot.q
        newR = state.r+dT*dot.r

        # Prepare the state to be returned
        newState = States.vehicleState(pn=newPn, pe=newPe, pd=newPd, u=newU, v=newV, w=newW, yaw=newYaw, pitch=newPitch, roll=newRoll, p=newP, q=newQ, r=newR)
        newState.Va = state.Va
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.chi = math.atan2(dot.pe, dot.pn)

        return newState

    def Rexp(self, dT, state, dot):
        # Get pqr and pqr_dot to do the trapezoid
        pqr = [[state.p],[state.q],[state.r]]
        pqr_dot = [[dot.p],[dot.q],[dot.r]]

        # Do trapezoid to find omega
        omega = MatrixMath.matrixAdd(pqr,MatrixMath.matrixScalarMultiply(dT/2,pqr_dot))

        # Compute other matrices and values needed
        omega_cross = MatrixMath.matrixSkew(omega[0][0],omega[1][0],omega[2][0])
        omega_cross_sq = MatrixMath.matrixMultiply(omega_cross,omega_cross)
        w_abs = math.hypot(omega[0][0],omega[1][0],omega[2][0])

        # Use the approxiamation when w_abs is too small
        if w_abs <= 0.2:
            sin_blob = (dT)-(dT**3)*(w_abs**2)/6+(dT**5)*(w_abs**4)/120
            cos_blob = (dT**2)/2+(dT**4)*(w_abs**2)/24+(dT**6)*(w_abs**4)/720
        else:
            sin_blob = math.sin(w_abs*dT)/w_abs
            cos_blob = (1-math.cos(w_abs*dT))/(w_abs**2)

        # Define Identity matrix
        I = [[1,0,0],[0,1,0],[0,0,1]]

        # Put together the Rexp
        Rexp = MatrixMath.matrixAdd(I,MatrixMath.matrixSubtract(MatrixMath.matrixScalarMultiply(cos_blob,omega_cross_sq),MatrixMath.matrixScalarMultiply(sin_blob,omega_cross)))

        return Rexp

    def Update(self, forcesMoments):
        dot = self.derivative(self.state,forcesMoments)
        self.dot = dot
        self.state = self.ForwardEuler(forcesMoments)
        return

    """
    def derivative(self, state, forcesMoments):
        # Define matrices used in computation
        uvw = [[state.u],[state.v],[state.w]]
        R = state.R
        Force = [[forcesMoments.Fx],[forcesMoments.Fy],[forcesMoments.Fz]]
        Moment = [[forcesMoments.Mx],[forcesMoments.My],[forcesMoments.Mz]]
        omega = [[state.p],[state.q],[state.r]]
        neg_omega_cross = MatrixMath.matrixScalarMultiply(-1, MatrixMath.matrixSkew(omega[0][0],omega[1][0],omega[2][0]))

        # Calculate all the necessary dot matrices
        p_dot = MatrixMath.matrixMultiply(MatrixMath.matrixTranspose(R), uvw)
        uvw_dot = MatrixMath.matrixAdd(MatrixMath.matrixMultiply(neg_omega_cross,uvw),MatrixMath.matrixScalarMultiply(1/VPC.mass,Force))
        R_dot = MatrixMath.matrixMultiply(neg_omega_cross,R)
        theta_dot = MatrixMath.matrixMultiply([[1,math.sin(state.roll)*math.tan(state.pitch),math.cos(state.roll)*math.tan(state.pitch)],[0,math.cos(state.roll),-math.sin(state.roll)],[0,math.sin(state.roll)/math.cos(state.pitch),math.cos(state.roll)/math.cos(state.pitch)]],omega)
        pqr_dot = MatrixMath.matrixMultiply(VPC.JinvBody, MatrixMath.matrixAdd(Moment, MatrixMath.matrixMultiply(MatrixMath.matrixMultiply(neg_omega_cross,VPC.Jbody), omega)))

        # Prepare the state to be returned
        returnState = States.vehicleState(pn=p_dot[0][0],pe=p_dot[1][0],pd=p_dot[2][0],u=uvw_dot[0][0],v=uvw_dot[1][0],w=uvw_dot[2][0],yaw=theta_dot[2][0],pitch=theta_dot[1][0],roll=theta_dot[0][0],p=pqr_dot[0][0],q=pqr_dot[1][0],r=pqr_dot[2][0])
        # Add in the R
        returnState.R = R_dot

        return returnState"""
        
    def derivative(self, state, forcesMoments):
        """ Function to compute the derivative of the state given body frame forces an
         moments """
        d = States.vehicleState()
        wx = [[0, -1 * state.r, state.q],
              [state.r, 0, -1 * state.p],
              [-1 * state.q, state.p, 0]]
        """Rotation Matrix"""
        d.R = MatrixMath.matrixMultiply(MatrixMath.matrixScalarMultiply(-1, wx), state.R)
        """positions"""
        mat1 = MatrixMath.matrixTranspose(state.R)
        rolls = [[state.u], [state.v], [state.w]]
        positions = MatrixMath.matrixMultiply(mat1, rolls)
        d.pn = positions[0][0]
        d.pe = positions[1][0]
        d.pd = positions[2][0]

        """velocities"""
        accels = MatrixMath.matrixScalarMultiply(1/VPC.mass, [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]])
        mat2 = [[state.r*state.v - state.q*state.w],
                [state.p*state.w - state.r*state.u],
                [state.q*state.u - state.p*state.v]]
        velocs = MatrixMath.matrixAdd(mat2, accels)
        d.u = velocs[0][0]
        d.v = velocs[1][0]
        d.w = velocs[2][0]

        """Euler Angles"""
        mat3 = [[1, math.sin(state.roll)*math.tan(state.pitch), math.cos(state.roll)*math.tan(state.pitch)],
                [0, math.cos(state.roll), -1*math.sin(state.roll)],
                [0, math.sin(state.roll)/math.cos(state.pitch), math.cos(state.roll)/math.cos(state.pitch)]]
        eulers = MatrixMath.matrixMultiply(mat3, [[state.p], [state.q], [state.r]])
        d.roll = eulers[0][0]
        d.pitch = eulers[1][0]
        d.yaw = eulers[2][0]

        """Roll Rates"""
        mat4 = MatrixMath.matrixMultiply(VPC.Jbody, [[state.p], [state.q], [state.r]])
        mat5 = MatrixMath.matrixMultiply(wx, mat4)
        M = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]
        mat6 = MatrixMath.matrixSubtract(M, mat5)
        rates = MatrixMath.matrixMultiply(VPC.JinvBody, mat6)
        d.p = rates[0][0]
        d.q = rates[1][0]
        d.r = rates[2][0]

        d.Va = state.Va
        d.alpha = state.alpha
        d.beta = state.beta
        d.chi = state.chi

        return d

    def getVehicleState(self):
        # Returns state
        return self.state

    def reset(self):
        # Set both state and derivative to 0s
        self.state = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
        self.dot = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
        return

    def resetVehicleState(self):
        # Set state to all 0s
        self.state = States.vehicleState(pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0)
        return

    def setVehicleState(self, state):
        # Simply set state
        self.state = state
        return