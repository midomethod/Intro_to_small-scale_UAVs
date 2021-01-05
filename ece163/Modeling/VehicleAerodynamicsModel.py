import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel():
    # Implemented
    def __init__(self, initialSpeed=25.0, initialHeight=-100.0):
        # Need an instance of VDM
        self.vehicleDynamics = VehicleDynamicsModel.VehicleDynamicsModel()

        """
    self.VDM.state
    self.VDM.dT
    self.VDM.dot
    all exist
    """
        # Need an instance of Wind model
        self.windModel = WindModel.WindModel()
        """
    self.WM.WS
    self.WM.dT
    self.WM.Va
    self.WM.dp
    """
        #self.drydenParameters = Inputs.drydenParameters()
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.vehicleDynamics.state.u = self.initialSpeed
        self.vehicleDynamics.state.pd = self.initialHeight
        self.wind = States.windState()
        self.state = self.vehicleDynamics.getVehicleState()
        self.dot = self.vehicleDynamics.dot

        return

    """
  # Implemented
  def CalculateAirspeed(self, state, wind):
      # The ground speed and wind vector
      Vg = [[state.u],[state.v],[state.w]]
      W_gust = [[wind.Wu],[wind.Wv],[wind.Ww]] # In wind frame
      chi = math.atan2(wind.We,wind.Wn)
      gamma = -math.atan2(wind.Wd,math.hypot(wind.Wn,wind.We)) 
      azi_ele = [[math.cos(chi)*math.cos(gamma),math.sin(chi)*math.cos(gamma),\
          -math.sin(gamma)],[-math.sin(chi),math.cos(chi),0],\
              [math.cos(chi)*math.sin(gamma),math.sin(chi)*math.sin(gamma),math.cos(gamma)]]
      Wg_I = MatrixMath.matrixMultiply(MatrixMath.matrixTranspose(azi_ele),W_gust)
      W_static_I = [[wind.Wn],[wind.We],[wind.Wd]]
      # Transfer wind to body frame
      W_body = MatrixMath.matrixMultiply(state.R,MatrixMath.matrixAdd(W_static_I,Wg_I))
      # Airspeed in body frame
      Va_vec = MatrixMath.matrixSubtract(Vg, W_body)

      # Calculate airspeed and angles
      Va = math.hypot(Va_vec[0][0],Va_vec[1][0],Va_vec[2][0])
      alpha = math.atan2(Va_vec[2][0],Va_vec[0][0])
      beta = math.asin(Va_vec[1][0]/Va)

      # Have different return values just in case
      if Va==0:
          return Va, 0, 0
      else:
          return Va, alpha, beta
  """

    def getDynamicsModel(self):
        return self.vehicleDynamics

    def CalculateAirspeed(self, state, wind):

        W = [[wind.Wn], [wind.We], [wind.Wd]]
        Xw = math.atan2(wind.We, wind.Wn)
        if math.isclose(math.hypot(wind.Wn, wind.We, wind.Wd), 0.0):
            Yw = 0
        else:
            Yw = -math.asin((wind.Wd) / math.hypot(wind.Wn, wind.We, wind.Wd))

        Rxy = [[math.cos(Xw) * math.cos(Yw), math.sin(Xw) * math.cos(Yw), -math.sin(Yw)],
               [-math.sin(Xw), math.cos(Xw), 0],
               [math.cos(Xw) * math.sin(Yw), math.sin(Xw) * math.sin(Yw), math.cos(Yw)]]

        RxyT = MatrixMath.matrixTranspose(Rxy)

        mat1 = MatrixMath.matrixMultiply(RxyT, [[wind.Wu], [wind.Wv], [wind.Ww]])
        mat2 = MatrixMath.matrixAdd(W, mat1)
        Wb = MatrixMath.matrixMultiply(state.R, mat2)

        airVector = [[state.u - Wb[0][0]],
                     [state.v - Wb[1][0]],
                     [state.w - Wb[2][0]]]

        self.vehicleDynamics.state.Va = math.hypot(airVector[0][0], airVector[1][0], airVector[2][0])
        self.vehicleDynamics.state.alpha = math.atan2(airVector[2][0], airVector[0][0])
        self.vehicleDynamics.state.beta = math.asin(
            (airVector[1][0]) / math.hypot(airVector[0][0], airVector[1][0], airVector[2][0]))

        if math.isclose(self.vehicleDynamics.state.Va, 0.0):
            self.vehicleDynamics.state.beta = 0.0

        return [self.vehicleDynamics.state.Va, self.vehicleDynamics.state.alpha, self.vehicleDynamics.state.beta]

    # Implemented
    def CalculateCoeff_alpha(self, alpha):
        # Construct sigmoid
        M = VPC.M
        a0 = VPC.alpha0
        sigmoid = (1 + math.exp(-M * (alpha - a0)) + math.exp(M * (alpha + a0))) / (
                    (1 + math.exp(-M * (alpha - a0))) * (1 + math.exp(M * (alpha + a0))))

        # Approximations and blending for Lift constant
        CL_lin = VPC.CL0 + VPC.CLalpha * alpha
        CL_aprx = 2 * math.sin(alpha) * math.cos(alpha)
        CL_alpha = (1 - sigmoid) * CL_lin + (sigmoid) * CL_aprx

        # Approximations and blending for Drag constant
        CD_par = VPC.CDp + ((CL_alpha * alpha) ** 2) / (math.pi * VPC.e * VPC.AR)
        CD_aprx = 2 * (math.sin(alpha) ** 2)
        CD_alpha = (1 - sigmoid) * CD_par + (sigmoid) * CD_aprx

        # Linear for CM
        CM_alpha = VPC.CM0 + VPC.CMalpha * alpha
        return CL_alpha, CD_alpha, CM_alpha

    # Implemented
    def CalculatePropForces(self, Va, Throttle):
        # Find requisite values for omega
        Kt = 60 / (2 * math.pi * VPC.KV)
        Ke = Kt
        Vin = VPC.V_max * Throttle
        a = VPC.rho * (VPC.D_prop ** 5) * VPC.C_Q0 / (4 * (math.pi ** 2))
        b = VPC.rho * (VPC.D_prop ** 4) * Va * VPC.C_Q1 / (2 * math.pi) + Kt * Ke / VPC.R_motor
        c = VPC.rho * (VPC.D_prop ** 3) * (Va ** 2) * VPC.C_Q2 - Kt * Vin / VPC.R_motor + Kt * VPC.i0

        # Find omega and J
        if a == 0:
            omega = -c / b
        elif 4 * a * c > b ** 2:
            omega = 100
        else:
            omega = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        J = 2 * math.pi * Va / (omega * VPC.D_prop)

        # Find CT and CQ
        CT = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * (J ** 2)
        CQ = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * (J ** 2)

        # Find the propeller forces and moments
        Fx_prop = VPC.rho * (omega ** 2) * (VPC.D_prop ** 4) * CT / (4 * math.pi ** 2)
        Mx_prop = -VPC.rho * (omega ** 2) * (VPC.D_prop ** 5) * CQ / (4 * math.pi ** 2)
        return Fx_prop, Mx_prop

    # Implemented
    def Update(self, controls):
        # Update Forces
        FM = self.updateForces(self.vehicleDynamics.state, self.windModel.Wind, controls)
        # Update VDM
        self.vehicleDynamics.Update(FM)
        # Update wind
        self.windModel.Update()
        return

    # Implemented
    def aeroForces(self, state):
        """wind = self.WM.WS
    Va, alpha, beta = self.CalculateAirspeed(state,wind)"""
        # Extract the airspeed values
        Va = state.Va
        alpha = state.alpha
        beta = state.beta

        if Va == 0:
            return Inputs.forcesMoments(0, 0, 0, 0, 0, 0)  # Forces Moments is 0 when no Va
        else:
            # Get coefficients
            CL_a, CD_a, CM_a = self.CalculateCoeff_alpha(alpha)

            # Get some variables from VPC
            p = state.p
            q = state.q
            r = state.r
            CLq = VPC.CLq
            CDq = VPC.CDq
            CMq = VPC.CMq
            c = VPC.c

            # Calculate total coefiicient
            CL_tot = CL_a + (CLq * c * q) / (2 * Va)
            CD_tot = CD_a + (CDq * c * q) / (2 * Va)
            CM_tot = CM_a + (CMq * c * q) / (2 * Va)

            # Get some more constants from VPC
            S = VPC.S
            rho = VPC.rho
            b = VPC.b
            cl0 = VPC.Cl0
            clbeta = VPC.Clbeta
            clp = VPC.Clp
            clr = VPC.Clr
            cn0 = VPC.Cn0
            cnbeta = VPC.Cnbeta
            cnp = VPC.Cnp
            cnr = VPC.Cnr
            # p_hat = b*p/(2*Va)
            # r_hat = b*r/(2*Va)

            # Find forces and moments in all directions
            FL = 0.5 * rho * (Va ** 2) * S * CL_tot
            FD = 0.5 * rho * (Va ** 2) * S * CD_tot
            fxfz = MatrixMath.matrixMultiply([[math.cos(alpha), -math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]],
                                             [[-FD], [-FL]])
            fy = 0.5 * rho * (Va ** 2) * S * (
                        VPC.CY0 + VPC.CYbeta * beta + VPC.CYp * b * p / (2 * Va) + VPC.CYr * b * r / (2 * Va))
            L = 0.5 * rho * (Va ** 2) * S * b * (cl0 + clbeta * beta + clp * b * p / (2 * Va) + clr * b * r / (2 * Va))
            M = 0.5 * rho * (Va ** 2) * S * c * CM_tot
            N = 0.5 * rho * (Va ** 2) * S * b * (cn0 + cnbeta * beta + cnp * b * p / (2 * Va) + cnr * b * r / (2 * Va))

            # Return ForcesMoments class
            return Inputs.forcesMoments(fxfz[0][0], fy, fxfz[1][0], L, M, N)  # Forces Moments

    # Implemented
    def controlForces(self, state, controls):
        # Get some variables from state and controls
        Va = state.Va
        alpha = state.alpha
        Thro = controls.Throttle
        Elev = controls.Elevator
        Aile = controls.Aileron
        Rudd = controls.Rudder

        # Get propeller forces
        Fxp, Mxp = self.CalculatePropForces(Va, Thro)

        # Calculate lift and drag due to control surfaces
        FL = 0.5 * VPC.rho * (state.Va ** 2) * VPC.S * (VPC.CLdeltaE * Elev)
        FD = 0.5 * VPC.rho * (state.Va ** 2) * VPC.S * (VPC.CDdeltaE * Elev)
        # Then convert it into body frame from stability frame
        fxfz = MatrixMath.matrixMultiply([[math.cos(alpha), -math.sin(alpha)], [math.sin(alpha), math.cos(alpha)]],
                                         [[-FD], [-FL]])
        fy = 0.5 * VPC.rho * (Va ** 2) * VPC.S * (VPC.CYdeltaA * Aile + VPC.CYdeltaR * Rudd)

        # Calculate moments due to control surafaces
        L = 0.5 * VPC.rho * (Va ** 2) * VPC.S * VPC.b * (VPC.CldeltaR * Rudd + VPC.CldeltaA * Aile)
        M = 0.5 * VPC.rho * (Va ** 2) * VPC.S * VPC.c * (VPC.CMdeltaE * Elev)
        N = 0.5 * VPC.rho * (Va ** 2) * VPC.S * VPC.b * (VPC.CndeltaR * Rudd + VPC.CndeltaA * Aile)

        # Return the sum of propeller forces+controlsurfaces
        return Inputs.forcesMoments(fxfz[0][0] + Fxp, fy, fxfz[1][0], L + Mxp, M, N)

    # Implemented
    def getVehicleState(self):
        return self.vehicleDynamics.state

    # Implemented
    def getWindState(self):
        return self.windModel.Wind

    # Implemented
    def gravityForces(self, state):
        # Find gravity and transform it into body frame
        F_g = [[0], [0], [VPC.mass * VPC.g0]]
        F_g = MatrixMath.matrixMultiply(state.R, F_g)
        return Inputs.forcesMoments(F_g[0][0], F_g[1][0], F_g[2][0], 0, 0, 0)

    # Implemented
    def reset(self):
        # Reset vehicle state and wind state
        self.vehicleDynamics.setVehicleState(
            States.vehicleState(pn=0.0, pe=0.0, pd=self.initialHeight, u=self.initialSpeed, v=0.0, w=0.0, yaw=0.0, pitch=0.0, roll=0.0, p=0.0, q=0.0, r=0.0))
        self.windModel.setWind(States.windState())
        return

    # Implemented
    def setVehicleState(self, state):
        self.vehicleDynamics.setVehicleState(state)
        return

    # Implemented
    def setWindModel(self, Wn=0.0, We=0.0, Wd=0.0,
                     drydenParamters=Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw=0.0, sigmau=0.0, sigmav=0.0,
                                                             sigmaw=0.0)):
        # Make new wind model
        self.windModel = WindModel.WindModel(
            drydenParamters=Inputs.drydenParameters(Lu=0.0, Lv=0.0, Lw=0.0, sigmau=0.0, sigmav=0.0, sigmaw=0.0))
        # Then set the wind parameters
        self.windModel.Wind.Wn = Wn
        self.windModel.Wind.We = We
        self.windModel.Wind.Wd = Wd
        return

    def updateForces(self, state, wind, controls):
        # First update Va
        Va, alpha, beta = self.CalculateAirspeed(state, wind)
        state.Va = Va
        state.alpha = alpha
        state.beta = beta
        Va, alpha, beta = self.CalculateAirspeed(state, wind)
        self.vehicleDynamics.state.Va = Va
        self.vehicleDynamics.state.alpha = alpha
        self.vehicleDynamics.state.beta = beta



        # Then calculate forces
        aero = self.aeroForces(state)
        ctrl = self.controlForces(state, controls)
        grav = self.gravityForces(state)

        # Then add forces
        fx = aero.Fx + ctrl.Fx + grav.Fx
        fy = aero.Fy + ctrl.Fy + grav.Fy
        fz = aero.Fz + ctrl.Fz + grav.Fz
        mx = aero.Mx + ctrl.Mx + grav.Mx
        my = aero.My + ctrl.My + grav.My
        mz = aero.Mz + ctrl.Mz + grav.Mz

        # Then return forces
        return Inputs.forcesMoments(Fx=fx, Fy=fy, Fz=fz, Mx=mx, My=my, Mz=mz)
