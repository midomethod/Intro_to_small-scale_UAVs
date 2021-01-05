import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC

# Gauss-Markov process used for the gyro measurement and GPS
class GaussMarkov():

    def __init__(self, dT=VPC.dT, tau=1e6, eta=0.0):
        self.bias = 0
        self.dT = dT
        self.tau = tau
        self.eta = eta
        return

    def reset(self):
        self.bias = 0
        return

    def update(self, vnoise=None):
        # If noise is not passed in manually, defaults to gaussian distribution
        if vnoise is None:
            wnoise = random.gauss(0,self.eta)
        else:
            wnoise = vnoise

        self.bias = self.bias*math.exp(-self.dT/self.tau)+wnoise
        return self.bias

# Just uses three instances of the above class
class GaussMarkovXYZ():

    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        self.GM_x = GaussMarkov(dT,tauX,etaX)
        # We need to trap the case where the vars for y and z aren't specified and set it to etaX and tauX
        if tauY is None:
            ty = tauX
        else:
            ty = tauY
        if tauZ is None:
            tz = tauX
        else:
            tz = tauZ
        if etaY is None:
            ey = etaX
        else:
            ey = etaY
        if etaZ is None:
            ez = etaX
        else:
            ez = etaZ
        self.GM_y = GaussMarkov(dT,ty,ey)
        self.GM_z = GaussMarkov(dT,tz,ez)
        return

    def reset(self):
        self.GM_x.reset()
        self.GM_y.reset()
        self.GM_z.reset()
        return

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        self.GM_x.update(vXnoise)
        self.GM_y.update(vYnoise)
        self.GM_z.update(vZnoise)
        return self.GM_x.bias,self.GM_y.bias,self.GM_z.bias

class SensorsModel():
    # Models the noisy sensor outputs based on true positions
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro=VSC.gyro_tau, etagyro=VSC.gyro_eta, tauGPS=VSC.GPS_tau, etaGPSHorizontal=VSC.GPS_etaHorizontal,etaGPSVertical=VSC.GPS_etaVertical, gpsUpdateHz=VSC.GPS_rate):
        self.aeroModel = aeroModel
        VD = self.aeroModel.getDynamicsModel()
        self.dT = VD.dT

        # Carry the previous true and noisy sensors to allow for GPS zero degree hold
        self.trueS = Sensors.vehicleSensors()
        self.noisyS = Sensors.vehicleSensors()

        # Initialize the biases in here
        self.sensorBias = self.initializeBiases()
        self.sensorSigma = self.initializeSigmas()

        # Have instances of gaussMarkov for both gps and gyro
        self.GPS_GauMar = GaussMarkovXYZ(self.dT, tauX=tauGPS, etaX=etaGPSHorizontal, tauY=tauGPS, etaY=etaGPSHorizontal, tauZ=tauGPS, etaZ=etaGPSVertical)
        self.GYR_GauMar = GaussMarkovXYZ(self.dT, tauX=taugyro, etaX=etagyro, tauY=taugyro, etaY=etagyro, tauZ=taugyro, etaZ=etagyro)
        
        # Count the ticks to delay the update rate for GPS
        self.GPS_tick = 0
        self.GPS_max = 1/(self.dT*gpsUpdateHz)
        return
    
    # Simple getters
    def getSensorsNoisy(self):
        return self.noisyS

    def getSensorsTrue(self):
        return self.trueS
    
    # Initialize the biases, but scaled by a uniform 
    def initializeBiases(self, gyroBias=VSC.gyro_bias , accelBias=VSC.accel_bias, magBias=VSC.mag_bias, baroBias=VSC.baro_bias, pitotBias=VSC.pitot_bias):
        
        # An empty instance to be filled
        returnS = Sensors.vehicleSensors()

        # Filling it up
        returnS.gyro_x = gyroBias*random.uniform(-1,1)
        returnS.gyro_y = gyroBias*random.uniform(-1,1)
        returnS.gyro_z = gyroBias*random.uniform(-1,1)
        returnS.accel_x = accelBias*random.uniform(-1,1)
        returnS.accel_y = accelBias*random.uniform(-1,1)
        returnS.accel_z = accelBias*random.uniform(-1,1)
        returnS.mag_x = magBias*random.uniform(-1,1)
        returnS.mag_y = magBias*random.uniform(-1,1)
        returnS.mag_z = magBias*random.uniform(-1,1)
        returnS.baro = baroBias*random.uniform(-1,1)
        returnS.pitot = pitotBias*random.uniform(-1,1)
        
        return returnS
    
    # Same deal as the biases
    def initializeSigmas(self, gyroSigma=VSC.gyro_sigma, accelSigma=VSC.accel_sigma, magSigma=VSC.mag_sigma, baroSigma=VSC.baro_sigma, pitotSigma=VSC.pitot_sigma, gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal, gpsSigmaVertical=VSC.GPS_sigmaVertical, gpsSigmaSOG=VSC.GPS_sigmaSOG, gpsSigmaCOG=VSC.GPS_sigmaCOG):
        returnS = Sensors.vehicleSensors()

        returnS.gyro_x = gyroSigma
        returnS.gyro_y = gyroSigma
        returnS.gyro_z = gyroSigma
        returnS.accel_x = accelSigma
        returnS.accel_y = accelSigma
        returnS.accel_z = accelSigma
        returnS.mag_x = magSigma
        returnS.mag_y = magSigma
        returnS.mag_z = magSigma
        returnS.baro = baroSigma
        returnS.pitot = pitotSigma
        returnS.gps_n = gpsSigmaHorizontal
        returnS.gps_e = gpsSigmaHorizontal
        returnS.gps_alt = gpsSigmaVertical
        returnS.gps_cog = gpsSigmaCOG
        returnS.gps_sog = gpsSigmaSOG

        return returnS

    # Reset all of the noisy processes
    def reset(self):
        self.GPS_iterate = 0
        self.GYR_GauMar.reset()
        self.GPS_GauMar.reset()
        self.trueS = Sensors.vehicleSensors()
        self.noisyS = Sensors.vehicleSensors()
        self.sensorBias = self.initializeBiases()
        self.sensorSigma = self.initializeSigmas()
        return

    # Just a wrapper to update everything
    def update(self):
        state = self.aeroModel.vehicleDynamics.state    
        dot = self.aeroModel.vehicleDynamics.dot

        self.trueS = self.updateSensorsTrue(self.trueS, state, dot)
        self.noisyS = self.updateSensorsNoisy(self.trueS,self.noisyS,self.sensorBias,self.sensorSigma)

        self.GPS_tick += 1

        return

    def updateAccelsTrue(self, state, dot):

        uvw =   [[state.u],
                [state.v],
                [state.w]]

        uvw_dot =   [[dot.u],
                    [dot.v],
                    [dot.w]]

        Wcross = MatrixMath.matrixSkew(state.p,state.q,state.r)

        g = [[0],[0],[-VPC.g0]]

        acc = MatrixMath.matrixAdd(uvw_dot,MatrixMath.matrixAdd(MatrixMath.matrixMultiply(Wcross,uvw),MatrixMath.matrixMultiply(state.R,g)))

        accX = acc[0][0]
        accY = acc[1][0]
        accZ = acc[2][0]
        return accX, accY, accZ

    def updateGPSTrue(self, state, dot):
        gps_n = state.pn
        gps_e = state.pe
        gps_alt = -state.pd
        gps_COG = math.atan2(dot.pe,dot.pn)
        gps_SOG = math.hypot(state.u,state.v,state.w)
        return gps_n, gps_e, gps_alt, gps_SOG, gps_COG

    def updateGyrosTrue(self, state):
        gyro_x =state.p
        gyro_y =state.q
        gyro_z =state.r
        return gyro_x, gyro_y, gyro_z

    def updateMagsTrue(self, state):
        mag_xyz = MatrixMath.matrixMultiply(state.R,VSC.magfield)
        return mag_xyz[0][0], mag_xyz[1][0], mag_xyz[2][0]

    def updatePressureSensorsTrue(self, state):
        baro = (state.pd)*VPC.rho*VPC.g0+VSC.Pground      
        pitot = 0.5*VPC.rho*(state.Va**2)
        return baro, pitot

    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(), noisySensors=Sensors.vehicleSensors(), sensorBiases=Sensors.vehicleSensors(), sensorSigmas=Sensors.vehicleSensors()):
        
        gyr_btx, gyr_bty, gyr_btz = self.GYR_GauMar.update()

        returnS = Sensors.vehicleSensors()
        
        if self.GPS_tick%self.GPS_max != 0: # Not ready to update
            returnS.gps_n = noisySensors.gps_n
            returnS.gps_e = noisySensors.gps_e
            returnS.gps_alt = noisySensors.gps_alt
            returnS.gps_sog = noisySensors.gps_sog
            returnS.gps_cog = noisySensors.gps_cog
        else: # Ready to update
            gps_btx, gps_bty, gps_btz = self.GPS_GauMar.update()

            returnS.gps_n = trueSensors.gps_n + gps_btx + random.gauss(0,sensorSigmas.gps_n)
            returnS.gps_e = trueSensors.gps_e + gps_bty + random.gauss(0,sensorSigmas.gps_e)
            returnS.gps_alt = trueSensors.gps_alt + gps_btz + random.gauss(0,sensorSigmas.gps_alt)

            returnS.gps_sog = trueSensors.gps_sog + random.gauss(0,sensorSigmas.gps_sog)
            if math.isclose(0,returnS.gps_sog):
                returnS.gps_cog = trueSensors.gps_cog + random.gauss(0,100*sensorSigmas.gps_cog)
            else:
                returnS.gps_cog = trueSensors.gps_cog + random.gauss(0,sensorSigmas.gps_cog*VPC.InitialSpeed/sensorSigmas.gps_sog)
        
        returnS.gyro_x = trueSensors.gyro_x + gyr_btx + sensorBiases.gyro_x + random.gauss(0,sensorSigmas.gyro_x)
        returnS.gyro_y = trueSensors.gyro_y + gyr_bty + sensorBiases.gyro_y + random.gauss(0,sensorSigmas.gyro_y)
        returnS.gyro_z = trueSensors.gyro_z + gyr_btz + sensorBiases.gyro_z + random.gauss(0,sensorSigmas.gyro_z)

        returnS.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0,sensorSigmas.mag_x)
        returnS.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0,sensorSigmas.mag_y)
        returnS.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0,sensorSigmas.mag_z)

        returnS.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0,sensorSigmas.accel_x)
        returnS.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0,sensorSigmas.accel_y)
        returnS.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0,sensorSigmas.accel_z)

        returnS.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0,sensorSigmas.baro)
        returnS.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0,sensorSigmas.pitot)

        return returnS

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        returnS = Sensors.vehicleSensors()

        if self.GPS_tick%self.GPS_max != 0:
            returnS.gps_n = prevTrueSensors.gps_n
            returnS.gps_e = prevTrueSensors.gps_e
            returnS.gps_alt = prevTrueSensors.gps_alt
            returnS.gps_sog = prevTrueSensors.gps_sog
            returnS.gps_cog = prevTrueSensors.gps_cog
        else:
            returnS.gps_n, returnS.gps_e, returnS.gps_alt, returnS.gps_sog, returnS.gps_cog = self.updateGPSTrue(state,dot)
        
        returnS.gyro_x, returnS.gyro_y, returnS.gyro_z = self.updateGyrosTrue(state)
        returnS.mag_x, returnS.mag_y, returnS.mag_z = self.updateMagsTrue(state)
        returnS.accel_x, returnS.accel_y, returnS.accel_z = self.updateAccelsTrue(state,dot)
        returnS.baro, returnS.pitot = self.updatePressureSensorsTrue(state)

        return returnS
