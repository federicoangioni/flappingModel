import os
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from analysis_utils import LogReader, rotation_matrix
from plot_settings import *

save_figures = False

DIR_DATA = os.path.join(os.path.dirname(__file__), 'data/velocity_model')

TRAINING_FILES = [
    "fd_model_tuning_level_flight_1.csv",
    "fd_model_tuning_level_flight_2.csv",
    "fd_model_tuning_level_flight_3.csv",
]

TESTING_FILES = [
    "fd_model_tuning_wAlt_1.csv",
    "fd_model_tuning_wAlt_2.csv",
    "fd_model_tuning_wAlt_3.csv"
]

DEG2RAD = math.pi/180
G = 9.81


def pseudoInverse(M):
    M = np.array(M)
    MT = np.transpose(M)
    MTM = np.matmul(MT, M)
    if isinstance(MTM, float):
        MTM_i = 1/MTM
        M_inv = MTM_i * MT
    else:
        MTM_i = np.linalg.inv(MTM)
        M_inv = np.matmul(MTM_i, MT)
    return M_inv

class VzComplementaryFilter:
    def __init__(self, Td, k1, k2) -> None:
        # discretization using tustin method in the form
        # G(z) = (n[2]*z^-2 + n[1]*z^-1 + n[2]) / (d[2]*z^-2 + d[1]*z^-1 + 1)
        # G1 =  s/(s^2 + k1*s + k2)
        self.G1_num = np.array([
            2*Td, 
            0, 
            -2*Td])
        self.G1_den = np.array([
            4 + 2*k1*Td + k2*Td**2,
            -8+2*k2*Td**2,
            4 - 2*k1*Td + k2*Td**2
        ])
        self.G1_num = self.G1_num/self.G1_den[0]
        self.G1_den = self.G1_den/self.G1_den[0]

        # G2 =  s(k1*s + k2)/(s^2 + k1*s + k2)
        self.G2_num = np.array([
            4*k1 + 2*k2*Td, 
            -8*k1, 
            4*k1 - 2*k2*Td])
        self.G2_den = np.array([
            4 + 2*k1*Td + k2*Td**2,
            -8+2*k2*Td**2,
            4 - 2*k1*Td + k2*Td**2
        ])
        self.G2_num = self.G2_num/self.G2_den[0]
        self.G2_den = self.G2_den/self.G2_den[0]
        self.reset()

    def reset(self):
        self.u1_d = 0
        self.u1_dd = 0
        self.u2_d = 0
        self.u2_dd = 0
        self.y_d = 0
        self.y_dd = 0

    def step(self, u1, u2):
        y = self.G1_num[2]*self.u1_dd + \
            self.G1_num[1]*self.u1_d+ \
            self.G1_num[0]*u1 + \
            self.G2_num[2]*self.u2_dd + \
            self.G2_num[1]*self.u2_d + \
            self.G2_num[0]*u2 - \
            self.G1_den[2]*self.y_dd - \
            self.G1_den[1]*self.y_d
        self.u1_dd = self.u1_d
        self.u1_d = u1
        self.u2_dd = self.u2_d
        self.u2_d = u2
        self.y_dd = self.y_d
        self.y_d = y
        
        return y

vz_filter = VzComplementaryFilter(0.01, k1=1.3, k2=0.02)

# Training
dt = 0.01
X = [[],[]]
Y = [[],[]]

for fname in TRAINING_FILES:
    f = os.path.join(DIR_DATA, fname)
    log = LogReader(f)
    log.prune_data(log.thrust>0)

    N = len(log.t)
    g = [0,0,-G]
    horizontal_v = np.zeros((3,N))

    for k in range(N):
        # calculate horizontal velocity groundtruth
        v = [log.ot[0].velocity[0][k], log.ot[0].velocity[1][k], log.ot[0].velocity[2][k]]
        yaw = log.ot[0].attitude[2][k]
        horizontal_v[:,k] = [
            v[0]*np.cos(yaw) + v[1]*np.sin(yaw), 
            -v[0]*np.sin(yaw) + v[1]*np.cos(yaw), 
            v[2]
        ]

    for k in range(N-1):
        if log.ot[0].position[2][k]<0.1:
            continue
        else:
            vx, vy, vz = horizontal_v[:,k]
            cth = np.cos(-log.ob.attitude[1][k])
            sth = np.sin(-log.ob.attitude[1][k])
            cph = np.cos(log.ob.attitude[0][k])
            sph = np.sin(log.ob.attitude[0][k])
            T = log.thrust[k] * log.vbat[k]

            Y[0].append(-(horizontal_v[0][k+1]-vx)/dt + G*sth/cth)
            X[0].append(cth*cth*vx)

            Y[1].append(-(horizontal_v[1][k+1]-vy)/dt - G*sph/cth/cph)
            X[1].append(cph*cph*vy)

b = [
    np.matmul(pseudoInverse(X[0]), Y[0]),
    np.matmul(pseudoInverse(X[1]), Y[1])
    ]

print("Parameters: bx={:.4f}, by={:.4f}".format(*b))

# Calculate error on training set
print("---------------------------------------------------")
rmse_train = [[],[],[]]
for fname in TRAINING_FILES:
    f = os.path.join(DIR_DATA, fname)
    log = LogReader(f)
    log.prune_data(log.thrust>0)

    N = len(log.t)
    horizontal_v = np.zeros((3,N))
    v_pred = np.zeros((3,N))

    vz_filter.reset()

    for k in range(N-1):
        # Groundtruth
        v = [log.ot[0].velocity[0][k], log.ot[0].velocity[1][k], log.ot[0].velocity[2][k]]
        yaw = log.ot[0].attitude[2][k]
        horizontal_v[:,k] = [
            v[0]*np.cos(yaw) + v[1]*np.sin(yaw), 
            -v[0]*np.sin(yaw) + v[1]*np.cos(yaw), 
            v[2]
        ]

        # on-board
        R = rotation_matrix(log.ob.attitude[0][k],log.ob.attitude[1][k],log.ob.attitude[2][k])
        g_rot = [R[2][0]*G, R[2][1]*G, R[2][2]*G]   # R_inv*g

        dt = log.t[k+1] - log.t[k]

        cth = np.cos(-log.ob.attitude[1][k])
        sth = np.sin(-log.ob.attitude[1][k])
        cph = np.cos(log.ob.attitude[0][k])
        sph = np.sin(log.ob.attitude[0][k])

        horizontal_acc = [cth*log.acc[0][k+1] + sth*sph*log.acc[1][k+1] + sth*cph*log.acc[2][k+1],
                        cph*log.acc[1][k+1] - sph*log.acc[2][k+1],
                        -sth*log.acc[0][k+1] + cth*sph*log.acc[1][k+1] + cph*cth*log.acc[2][k+1]]


        ax = G*sth/cth-b[0]*cth*cth*v_pred[0][k]
        ay = -G*sph/cth/cph-b[1]*cph*cph*v_pred[1][k]
        v_pred[0][k+1] = v_pred[0][k] + dt*ax
        v_pred[1][k+1] = v_pred[1][k] + dt*ay
        v_pred[2][k+1] = vz_filter.step(horizontal_acc[2]-G, log.baro[k+1])

    v_error = horizontal_v - v_pred
    rmse_train[0].append(np.sqrt(np.mean(v_error[0]**2)))
    rmse_train[1].append(np.sqrt(np.mean(v_error[1]**2)))
    rmse_train[2].append(np.sqrt(np.mean(v_error[2]**2)))

mean_rmse = np.mean(rmse_train, axis=1)
print("Mean RMSE Training: x={:.4f}, y={:.4f}, z={:.4f}".format(*mean_rmse))

# Plot for last file
fig, ax = plt.subplots(3,1)
ax[0].plot(log.t, horizontal_v[0], label='Optitrack', linewidth=1)
ax[0].plot(log.t, v_pred[0], label='Estimate', linewidth=1)
ax[0].set_ylabel('vx [m/s]')
ax[1].plot(log.t, horizontal_v[1], label='Optitrack', linewidth=1)
ax[1].plot(log.t, v_pred[1], label='Estimate', linewidth=1)
ax[1].set_ylabel('vy [m/s]')
ax[2].plot(log.t, horizontal_v[2], label='Optitrack', linewidth=1)
ax[2].plot(log.t, v_pred[2], label='Estimate', linewidth=1)
ax[2].set_ylabel('vz [m/s]')
ax[2].set_xlabel('t [s]')
ax[2].legend(loc='lower left')
for a in ax:
    a.set_ylim(-2.5, 2.5)
    a.grid(True)

if save_figures:
    save_name = 'fd_vel_training.eps'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='eps')

# Calculate error on testing set
print("---------------------------------------------------")
rmse_test = [[],[],[]]
for fname in TESTING_FILES:
    f = os.path.join(DIR_DATA, fname)
    log = LogReader(f)
    log.prune_data(log.thrust>0)

    N = len(log.t)
    horizontal_v = np.zeros((3,N))
    v_pred = np.zeros((3,N))

    vz_filter.reset()

    for k in range(N-1):
        # Groundtruth
        v = [log.ot[0].velocity[0][k], log.ot[0].velocity[1][k], log.ot[0].velocity[2][k]]
        yaw = log.ot[0].attitude[2][k]
        horizontal_v[:,k] = [
            v[0]*np.cos(yaw) + v[1]*np.sin(yaw), 
            -v[0]*np.sin(yaw) + v[1]*np.cos(yaw), 
            v[2]
        ]

        # on-board
        R = rotation_matrix(log.ob.attitude[0][k],log.ob.attitude[1][k],log.ob.attitude[2][k])
        g_rot = [R[2][0]*G, R[2][1]*G, R[2][2]*G]   # R_inv*g

        dt = log.t[k+1] - log.t[k]

        cth = np.cos(-log.ob.attitude[1][k])
        sth = np.sin(-log.ob.attitude[1][k])
        cph = np.cos(log.ob.attitude[0][k])
        sph = np.sin(log.ob.attitude[0][k])

        horizontal_acc = [cth*log.acc[0][k+1] + sth*sph*log.acc[1][k+1] + sth*cph*log.acc[2][k+1],
                        cph*log.acc[1][k+1] - sph*log.acc[2][k+1],
                        -sth*log.acc[0][k+1] + cth*sph*log.acc[1][k+1] + cph*cth*log.acc[2][k+1]]


        ax = G*sth/cth-b[0]*cth*cth*v_pred[0][k]
        ay = -G*sph/cth/cph-b[1]*cph*cph*v_pred[1][k]
        v_pred[0][k+1] = v_pred[0][k] + dt*ax
        v_pred[1][k+1] = v_pred[1][k] + dt*ay
        v_pred[2][k+1] = vz_filter.step(horizontal_acc[2]-G, log.baro[k+1])

    v_error = horizontal_v - v_pred
    rmse_test[0].append(np.sqrt(np.mean(v_error[0]**2)))
    rmse_test[1].append(np.sqrt(np.mean(v_error[1]**2)))
    rmse_test[2].append(np.sqrt(np.mean(v_error[2]**2)))

mean_rmse = np.mean(rmse_test, axis=1)
print("Mean RMSE Testing: x={:.4f}, y={:.4f}, z={:.4f}".format(*mean_rmse))

# Plot for last file
fig, ax = plt.subplots(3,1)
ax[0].plot(log.t, horizontal_v[0], label='Optitrack', linewidth=1)
ax[0].plot(log.t, v_pred[0], label='Estimate', linewidth=1)
ax[0].set_ylabel('vx [m/s]')
ax[1].plot(log.t, horizontal_v[1], label='Optitrack', linewidth=1)
ax[1].plot(log.t, v_pred[1], label='Estimate', linewidth=1)
ax[1].set_ylabel('vy [m/s]')
ax[2].plot(log.t, horizontal_v[2], label='Optitrack', linewidth=1)
ax[2].plot(log.t, v_pred[2], label='Estimate', linewidth=1)
ax[2].set_ylabel('vz [m/s]')
ax[2].set_xlabel('t [s]')
ax[2].legend(loc='lower left')
for a in ax:
    a.set_ylim(-2.5, 2.5)
    a.grid(True)

if save_figures:
    save_name = 'fd_vel_testing.eps'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='eps')
    
plt.show()