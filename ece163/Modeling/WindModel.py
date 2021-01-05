import math
import random
from ..Containers import States
from ..Containers import Inputs
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel():

    def __init__(self, dT=0.01, Va=25.0, drydenParamters= VPC.DrydenNoWind):
        self.Wind = States.windState()
        self.dT = dT
        self.Va = Va
        self.dp = drydenParamters
        self.x_u = [[0]]
        self.x_v = [[0],[0]]
        self.x_w = [[0],[0]]
        
        # Set it to a value for a moment
        self.CreateDrydenTransferFns(dT, Va, drydenParamters)
        return

    def CreateDrydenTransferFns(self, dT, Va, drydenParamters):
        # Get the values into some variables
        Lu=drydenParamters.Lu
        Lv=drydenParamters.Lv
        Lw=drydenParamters.Lw
        sigu=drydenParamters.sigmau
        sigv=drydenParamters.sigmav
        sigw=drydenParamters.sigmaw

        # Make variable for the U_gust transfer function
        if Lu==0:
            self.Phi_u = [[1]]
            self.Gamma_u = [[0]]
            self.H_u = [[1]]
        else:
            self.Phi_u = [[math.exp(-Va*dT/Lu)]]
            self.Gamma_u = [[(Lu/Va)*(1-math.exp(-Va*dT/Lu))]]
            self.H_u = [[sigu*math.sqrt(2*Va/(math.pi*Lu))]]

        # Make variables for the V_gust transfer functions
        if Lv==0:
            self.Phi_v = [[1,0],[0,1]]
            self.Gamma_v = [[0],[0]]
            self.H_v = [[1,1]]
        else:
            self.Phi_v = MatrixMath.matrixScalarMultiply(math.exp(-Va*dT/Lv),[[1-Va*dT/Lv,-dT*(Va/Lv)**2],[dT,1+Va*dT/Lv]]) 
            self.Gamma_v = MatrixMath.matrixScalarMultiply(math.exp(-Va*dT/Lv),[[dT],[(Lv/Va)**2*(math.exp(Va*dT/Lv)-1)-Lv*dT/Va]])
            self.H_v = MatrixMath.matrixScalarMultiply(sigv*math.sqrt(3*Va/(math.pi*Lv)),[[1,Va/(math.sqrt(3)*Lv)]])
        
        # Make variables for the W_gust transfer functions
        if Lw==0:
            self.Phi_w = [[1,0],[0,1]]
            self.Gamma_w = [[0],[0]]
            self.H_w = [[1,1]]
        else:
            self.Phi_w = MatrixMath.matrixScalarMultiply(math.exp(-Va*dT/Lw),[[1-Va*dT/Lw,-dT*(Va/Lw)**2],[dT,1+Va*dT/Lw]]) 
            self.Gamma_w = MatrixMath.matrixScalarMultiply(math.exp(-Va*dT/Lw),[[dT],[(Lw/Va)**2*(math.exp(Va*dT/Lw)-1)-Lw*dT/Va]])
            self.H_w = MatrixMath.matrixScalarMultiply(sigw*math.sqrt(3*Va/(math.pi*Lw)),[[1,Va/(math.sqrt(3)*Lw)]])
        return

    def Update(self, uu=None, uv=None, uw=None):
        """ Uses the Guassian to pass white noise through the TF and update wind"""
        # Use injected values, but use white noise otherwise
        if uu==None:
            in_u = random.gauss(0,1)
        else:
            in_u = uu
        if uv==None:
            in_v = random.gauss(0,1)
        else:
            in_v = uv
        if uw==None:
            in_w = random.gauss(0,1)
        else:
            in_w = uw
        
        # Update internal states using the gaussian or the injected value
        self.x_u = MatrixMath.matrixAdd(MatrixMath.matrixMultiply(self.Phi_u,self.x_u),MatrixMath.matrixScalarMultiply(in_u,self.Gamma_u))
        self.x_v = MatrixMath.matrixAdd(MatrixMath.matrixMultiply(self.Phi_v,self.x_v),MatrixMath.matrixScalarMultiply(in_v,self.Gamma_v))
        self.x_w = MatrixMath.matrixAdd(MatrixMath.matrixMultiply(self.Phi_w,self.x_w),MatrixMath.matrixScalarMultiply(in_w,self.Gamma_w))

        self.Wind.Wu = MatrixMath.matrixMultiply(self.H_u,self.x_u)[0][0]
        self.Wind.Wv = MatrixMath.matrixMultiply(self.H_v,self.x_v)[0][0]
        self.Wind.Ww = MatrixMath.matrixMultiply(self.H_w,self.x_w)[0][0]
        return

    def getDrydenTransferFns(self):
        return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w
        
    def getWind(self):
        return self.Wind

    def reset(self):
        self.Wind = States.windState()
        self.x_u = [[0]]
        self.x_v = [[0],[0]]
        self.x_w = [[0],[0]]
        return

    def setWind(self, windState):
        self.Wind = windState
        return
    