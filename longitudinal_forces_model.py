import numpy as np
import scipy
from scipy.signal import butter, filtfilt
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, lsim

# %% Define constants
g = 9.81
c1 = 0.0114
c2 = -0.0449
c_corr = 0.175
# To optimise for
Iyy = 1.00e-4
bx = 4.21e-3 # in paper
lz = 27e-3
m = 29.4e-3
lw = 81e-3
bz = 9.16e-4

# %% Results from regression 
"""
bx  = 0.0117566873, lw*bx = 0.0004477426
bz  = 0.0010171981, lw * bz = 0.00733434

"""

# %% Extract data from mat files
Nexp=102
nman=7
title_main = '360deg_pitch_maneuver_2'
title_comp = '360deg_pitch_maneuver_components'
save = False

# Figure 15 of Minimal Longitudinal uses experiment2, run 0
# checked up to 79
# 4, 13(?), 36 (almost sure), 54, 55(2, 4), 57(1)
mat = scipy.io.loadmat('dataset_revision.mat', squeeze_me=True, struct_as_record=False)

experiment_key = f"experiment{Nexp}"
data = mat[experiment_key]

# %% Extract necessary flight data
# Optitrack time
time = data.motion_tracking.TIME
time -=time[0]
thetadd_opti = np.radians(data.motion_tracking.ALPHy[nman])
thetadd_onboard = data.onboard.rates.ALPHy_IMU_filtered[nman]
time_onboard_rates = data.onboard.rates.TIME_onboard_rates[nman]
time_onboard_rates -=time_onboard_rates[0]
# Frequency, both onboard and interpolated + setpoints
ff = data.onboard_interpolated.FREQright_wing_interp[nman]
freq_right = data.onboard.frequency.FREQright_wing[nman]
time_freq = data.onboard.frequency.TIME_onboard_freq[nman]
CMDRight = data.onboard_interpolated.CMDthrottle_interp[nman]/4.1

# Accelerations
accx = data.motion_tracking.DVEL_BODYx_filtered[nman]
accz = data.motion_tracking.DVEL_BODYz_filtered[nman]
velx = data.motion_tracking.VEL_BODYx_filtered[nman]
velz = data.motion_tracking.VEL_BODYz_filtered[nman]

# Onboard time
time_onboard = data.onboard.angles_commands_setpoints.TIME_onboard[nman]
# ff = np.mean(data.onboard.frequency.FREQright_wing[nman])
time_onboard -= time_onboard[0]
# Dihedral commands and response
dihedral = data.motion_tracking.DIHEDRAL[nman]
CMD_dihed =np.radians(data.onboard_interpolated.CMDpitch_interp[nman]/100*18)

# %% Necessary to the model
pitch_raw = np.radians(data.onboard.angles_commands_setpoints.PITCH_IMU[nman])
pitch = interp1d(time_onboard, pitch_raw, fill_value='extrapolate')(time)
omy_raw = np.radians(data.onboard.rates.OMy_IMU_filtered[nman])
omy = interp1d(time_onboard_rates, omy_raw, fill_value='extrapolate')(time)

# Secondary inputs
# pitch = np.radians(data.motion_tracking.PITCH[nman])
# omy = np.radians(data.motion_tracking.OMy_filtered[nman])
# plt.plot(time, omy)
# plt.plot(time, omy_onboard)
# plt.show()

# %% Butterworth filter
fs = 1/np.mean(np.diff(time))
fc = 5
order = 4

b, a = butter(order, fc / (fs/2))
accz = filtfilt(b, a, accz)
accx = filtfilt(b, a, accx)
thetadd_onboard = filtfilt(b, a, thetadd_onboard)
thetadd_opti = filtfilt(b, a, thetadd_opti)

# %% Frequency transfer function 
tau = 0.0796
K = 1.0
numerator = [K]
denominator = [tau, 1]
system = TransferFunction(numerator, denominator)
t_ff, y_ff, _ = lsim(system, U=CMDRight, T=time)

# %% Dihedral transfer function
act_w0 = 40 # rad/s
act_damp = 0.634 # -
numerator = [act_w0**2]
denominator = [1, 2*act_damp*act_w0, act_w0**2]

system = TransferFunction(numerator, denominator)
t_dih, y_dih, _ = lsim(system, U=CMD_dihed, T=time)

# correction on dihedral due to velocity
dih_corr = -c_corr*velx + y_dih

# %% Force modeling
dt = np.mean(np.diff(time))
ld = lw*np.sin(dih_corr)
ldd = np.gradient(ld, time)

def T(f):
    return 2*(c1*f + c2)
fx = -np.sin(pitch)*g - y_ff * bx / m * (velx - lz*omy + ldd) - omy*velz
fz = np.cos(pitch)*g - y_ff * bz / m * (velz - ld*omy) + omy*velx - T(y_ff)/m
my = (-bx*y_ff * lz*(velx - lz*omy + ldd) + bz*y_ff*ld*(velz- ld*omy) - T(y_ff)*ld)/Iyy

# %% Plotting forces estimation

fig, axs = plt.subplots(5, 1, figsize= (8, 6))

axs[0].plot(time, accx)
axs[0].plot(time, fx, linestyle='--', color= 'black')
axs[0].set_ylabel(r'$\dot{u}$ [m/$s^2$]')
axs[0].set_xlim(0.5, 3.5)
axs[0].set_ylim(-20, 20)
axs[0].spines['top'].set_visible(False)
axs[0].spines['right'].set_visible(False)
axs[0].set_xticklabels([])

axs[1].plot(time, accz)
axs[1].plot(time, fz, linestyle='--', color= 'black')
axs[1].set_xlim(0.5, 3.5)
axs[1].set_ylim(-10, 35)
axs[1].set_ylabel(r'$\dot{w}$ [m/$s^2$]')
axs[1].spines['top'].set_visible(False)
axs[1].spines['right'].set_visible(False)
axs[1].set_xticklabels([])

axs[2].plot(time, thetadd_opti)
axs[2].plot(time, -my, linestyle='--', color= 'black')
axs[2].set_ylabel(r'$\ddot{\theta}$ [rad/s$^2$]')
axs[2].set_xlim(0.5, 3.5)
axs[2].set_ylim(-100, 100)
axs[2].spines['top'].set_visible(False)
axs[2].spines['right'].set_visible(False)
axs[2].set_xticklabels([])

axs[3].plot(time, dihedral, linewidth=1.0, label='Flight data')
axs[3].plot(t_dih, np.degrees(dih_corr), linestyle='--', color= 'black', label='Simulation')
axs[3].plot(time, np.degrees(CMD_dihed), linestyle='--', color= 'darkkhaki', label='Setpoint')
axs[3].set_ylabel(r'$\gamma_2 [deg]$')
axs[3].set_xlim(0.5, 3.5)
axs[3].spines['top'].set_visible(False)
axs[3].spines['right'].set_visible(False)
axs[3].set_xticklabels([])

axs[4].plot(time, ff, linewidth=1.0, label='Flight data')
axs[4].plot(t_ff, y_ff, linestyle='--', color='black', label='Simulation')
axs[4].plot(time, CMDRight, color= 'darkkhaki', linestyle='--', label='Setpoint')
axs[4].set_ylabel(r'f [Hz]')
axs[4].set_xlabel(r'Time [s]')
axs[4].set_xlim(0.5, 3.5)
axs[4].set_ylim(5.0, 27)
axs[4].spines['top'].set_visible(False)
axs[4].spines['right'].set_visible(False)
axs[4].legend(loc='upper left', bbox_to_anchor=(0.2, 0.8), fontsize = 8)

fig.align_ylabels(axs[:])
plt.tight_layout()

plt.suptitle('Using flight data to simulate accelerations')
if save:
    plt.savefig(f'results/{title_main}.png')

# %% Plotting components for forces estimation

fig, axs = plt.subplots(3, 2, figsize= (12, 4))

axs[0, 0].plot(time, velx, linewidth = 1.0, label='Flight data')
axs[0, 0].set_ylabel(r'u [m/s]')
axs[0, 0].set_xlim(0.5, 3.5)
axs[0, 0].spines['top'].set_visible(False)
axs[0, 0].spines['right'].set_visible(False)
axs[0, 0].set_xticklabels([])

axs[0, 1].plot(time, velz, linewidth = 1.0, label='Flight data')
axs[0, 1].set_ylabel(r'w [m/s]')
axs[0, 1].set_xlim(0.5, 3.5)
axs[0, 1].spines['top'].set_visible(False)
axs[0, 1].spines['right'].set_visible(False)
axs[0, 1].set_xticklabels([])

axs[1, 0].plot(time, ld, linewidth = 1.0, label='Flight data')
axs[1, 0].set_ylabel(r'$l_d$ [m]')
axs[1, 0].set_xlim(0.5, 3.5)
axs[1, 0].spines['top'].set_visible(False)
axs[1, 0].spines['right'].set_visible(False)
axs[1, 0].set_xticklabels([])


axs[1, 1].plot(time, ldd, linewidth = 1.0, label='Flight data')
axs[1, 1].set_ylabel(r'$l_{dd}$ [m/s]')
axs[1, 1].set_xlim(0.5, 3.5)
axs[1, 1].spines['top'].set_visible(False)
axs[1, 1].spines['right'].set_visible(False)
axs[1, 1].set_xticklabels([])

axs[2, 0].plot(time, pitch, linewidth = 1.0, label='Flight data')
axs[2, 0].set_ylabel(r'$\theta$ [rad]')
axs[2, 0].set_xlim(0.5, 3.5)
axs[2, 0].set_xlabel(r'Time [s]')
axs[2, 0].spines['top'].set_visible(False)
axs[2, 0].spines['right'].set_visible(False)

axs[2, 1].plot(time, omy, linewidth = 1.0, label='Flight data')
axs[2, 1].set_ylabel(r'$\dot{\theta}$ [rad/s]')
axs[2, 1].set_xlabel(r'Time [s]')
axs[2, 1].set_xlim(0.5, 3.5)
axs[2, 1].spines['top'].set_visible(False)
axs[2, 1].spines['right'].set_visible(False)
axs[2, 1].legend(fontsize = 8)

fig.align_ylabels(axs[:])
plt.suptitle('Dynamic components for longitudinal model')

if save:
    plt.savefig(f'results/{title_comp}.png')
    
# plt.show()