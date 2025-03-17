import os
import math
import pandas as pd
import numpy as np
import scipy.signal

from dataclasses import dataclass


DEG2RAD = math.pi/180
GRAVITY_MAGNITUDE = 9.81

@dataclass
class State:
    attitude: list
    position: list
    velocity: list

def lowpass_filter(data, cutoff, fs, order=4, edge_buffer=50):
    nyq = 0.5 * fs
    normal_cutoff = cutoff/nyq
    # buffer data to avoid edge effect
    if edge_buffer>0:
        data = np.block([data[0]*np.ones(edge_buffer), 
                        data, 
                        data[-1]*np.ones(edge_buffer)])
    sos = scipy.signal.butter(order, Wn=normal_cutoff, btype='low',
                            analog=False, output='sos')
    y = scipy.signal.sosfilt(sos, data)
    # remove buffer
    if edge_buffer>0:
        y = y[edge_buffer:-edge_buffer]
    return y

def lowpass_filter_fb(data, cutoff, fs, order=4, edge_buffer=50):
    # lowpass filter running on complete data forward and backward
    # only works for post-processing but has 0 phase
    nyq = 0.5 * fs
    normal_cutoff = cutoff/nyq
    # buffer data to avoid edge effect
    if edge_buffer>0:
        data = np.block([data[0]*np.ones(edge_buffer), 
                        data, 
                        data[-1]*np.ones(edge_buffer)])
    sos = scipy.signal.butter(order, Wn=normal_cutoff, btype='low',
                            analog=False, output='sos')
    y = scipy.signal.sosfiltfilt(sos, data)
    # remove buffer
    if edge_buffer>0:
        y = y[edge_buffer:-edge_buffer]
    return y

def central_diff(data, t):
    l = len(data)
    y = np.zeros(l)
    for i in range(l):
        try:
            y[i] = (data[i+1]-data[i-1])/(t[i+1]-t[i-1])
        except IndexError:
            if i == 0:
                y[i] = (data[i+1]-data[i])/(t[i+1]-t[i])
            else:
                y[i] = (data[i]-data[i-1])/(t[i]-t[i-1])
    return y


def rotation_matrix(roll, pitch, yaw):
    # v_G = R*v_B
    sphi = np.sin(roll)
    cphi = np.cos(roll)
    stheta = np.sin(-pitch)
    ctheta = np.cos(-pitch)
    spsi = np.sin(yaw)
    cpsi = np.cos(yaw)

    R = np.zeros((3,3))

    R[0][0] = cpsi*ctheta
    R[0][1] = cpsi*stheta*sphi - spsi*cphi
    R[0][2] = cpsi*stheta*cphi + spsi*sphi

    R[1][0] = spsi*ctheta
    R[1][1] = spsi*stheta*sphi + cpsi*cphi
    R[1][2] = spsi*stheta*cphi - cpsi*sphi

    R[2][0] = -stheta
    R[2][1] = ctheta*sphi
    R[2][2] = ctheta*cphi

    return R

def wrap_yaw(yaw):
    # wrap yaw so that it's always between -pi and pi
    for i in range(len(yaw)):
        while abs(yaw[i]) > math.pi:
            if yaw[i] >= math.pi:
                yaw[i] = yaw[i] - 2*math.pi
            elif yaw[i] <= math.pi:
                yaw[i] = yaw[i] + 2*math.pi
            else:
                break
    return yaw

class LogReader:
    def __init__(self, file) -> None:
        self.data = pd.read_csv(file, skipinitialspace=True)
        self.data = self.data.iloc[10:-10]
        self.load_data()

    def prune_data(self, bool_array):
        idx_true = np.nonzero(bool_array)
        first = idx_true[0][0]
        last = idx_true[0][-1]
        self.data = self.data.iloc[first:last+1]
        self.load_data()

    def load_data(self):
        # init frequent variables
        self.ob = State(None, None, None)  # On-board state estimate
        self.alt = State(None, None, None)  # Alternative state estimate
        self.ot = [State(None, None, None)]

        self.t = self.data["timeTick"].to_numpy()
        self.t = (self.t - self.t[0])/1000  # Convert to seconds & start at 0
        self.fs = 1/(self.t[1]-self.t[0])   # Sampling frequency
        
        # On-board estimate
        self._onboard_from_log()

        # Optitrack
        self._optitrack_from_log()

        # Alternative on-board estimate
        self._alternative_from_log()

        # IMU data
        self._imu_from_log()
        
        # Swarming data
        self._swarming_from_log()

        # Motor log group
        self.m = None
        try:
            self.m = np.array([self.data["m1"].to_numpy(),
                            self.data["m2"].to_numpy(),
                            self.data["m3"].to_numpy(),
                            self.data["m4"].to_numpy()])
        except KeyError:
            pass
        
        self.thrust = None
        try:
            self.thrust = self.data["thrust"].to_numpy()
        except KeyError:
            pass
        
        self.vbat = None
        try:
            self.vbat = self.data["vbat"].to_numpy()
        except KeyError:
            pass

        self.baro = None
        try:
            self.baro = self.data["baro"].to_numpy()
        except KeyError:
            pass


    def _imu_from_log(self):
        # Accelerometer
        self.acc = None
        try:
            self.acc = np.array([self.data["accX"].to_numpy() * GRAVITY_MAGNITUDE,
                                self.data["accY"].to_numpy() * GRAVITY_MAGNITUDE,
                                self.data["accZ"].to_numpy() * GRAVITY_MAGNITUDE])
        except KeyError:
            pass
        
        # Gyros
        self.gyro = None
        try:
            self.gyro = np.array([self.data["gyroX"].to_numpy() * DEG2RAD,
                                self.data["gyroY"].to_numpy() * DEG2RAD,
                                self.data["gyroZ"].to_numpy() * DEG2RAD])
        except KeyError:
            pass


    def _onboard_from_log(self):
        # Attitude
        try:
            self.ob.attitude = np.array([self.data["roll"].to_numpy() * DEG2RAD,
                                        self.data["pitch"].to_numpy() * DEG2RAD,
                                        self.data["yaw"].to_numpy() * DEG2RAD]) 

        except KeyError:
            pass
        
        # Position
        try:
            self.ob.position = np.array([self.data["stateX"].to_numpy(),
                                        self.data["stateY"].to_numpy(),
                                        self.data["stateZ"].to_numpy()])
        except KeyError:
            pass

        # Velocity
        try:
            self.ob.velocity = np.array([self.data["stateVX"].to_numpy(),
                                        self.data["stateVY"].to_numpy(),
                                        self.data["stateVZ"].to_numpy()])
        except KeyError:
            pass


    def _optitrack_from_log(self):
        # multiple drones
        for k in range(5):
            if "otX{}".format(k) in self.data.keys():
                if k>0:
                    self.ot.append(State(None, None, None))

                # Attitude
                try:
                    ot_att = [self.data["otRoll{}".format(k)].to_numpy() * DEG2RAD,
                                self.data["otPitch{}".format(k)].to_numpy() * DEG2RAD,
                                self.data["otYaw{}".format(k)].to_numpy() * DEG2RAD] 
                    self.ot[k].attitude = []
                    for i in range(3):
                        self.ot[k].attitude.append(lowpass_filter_fb(ot_att[i], 0.5, self.fs, order=4))
                except KeyError:
                    pass
                
                # Position & Velocity
                try:
                    ot_pos = np.array([self.data["otX{}".format(k)].to_numpy(),
                                        self.data["otY{}".format(k)].to_numpy(),
                                        self.data["otZ{}".format(k)].to_numpy()])
                    self.ot[k].position = []
                    self.ot[k].velocity = []
                    for i in range(3):
                        self.ot[k].position.append(lowpass_filter_fb(ot_pos[i], 0.5, self.fs, order=4))
                        self.ot[k].velocity.append(central_diff(self.ot[k].position[i], self.t))
                    self.ot[k].position = np.array(self.ot[k].position)
                    self.ot[k].velocity = np.array(self.ot[k].velocity)
                except KeyError:
                    pass
            else:
                break


    def _alternative_from_log(self):
        # Velocity
        try:
            self.alt.velocity = np.array([self.data["flapVX"].to_numpy(),
                                        self.data["flapVY"].to_numpy(),
                                        self.data["flapVZ"].to_numpy()])
        except KeyError:
            pass

        # Altitude
        try:
            self.alt.position = [0, 0, self.data["flapAlt"].to_numpy()]
        except KeyError:
            pass


    def _swarming_from_log(self):
        try:
            self.vel_0 = np.array([self.data["vx0"].to_numpy(),
                                    self.data["vy0"].to_numpy(),
                                    self.data["vz0"].to_numpy()])
            self.yawr_0 = self.data["r0"].to_numpy()

            self.vel_1 = np.array([self.data["vx1"].to_numpy(),
                                    self.data["vy1"].to_numpy(),
                                    self.data["vz1"].to_numpy()])
            self.yawr_1 = self.data["r1"].to_numpy()

            self.dist_1 = self.data["dist1"].to_numpy()
        
        except KeyError:
            pass
        
        try:
            self.relPos = np.array([self.data["rlX1"].to_numpy(),
                                    self.data["rlY1"].to_numpy(),
                                    self.data["rlZ1"].to_numpy(),
                                    self.data["rlYaw1"].to_numpy()])
        except KeyError:
            pass