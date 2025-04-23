import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt, savgol_filter

# %% Load the data
i = 3  # Index for first experiment (Python uses 0-based indexing)
j=0
mat = scipy.io.loadmat('dataset_revision.mat', squeeze_me=True, struct_as_record=False)

# Experimental setup
exp_title = 'PITCH_STEPS'
NEXP = [102, 101, 103, 94, 104]
legend_list = ['pitch 15°', 'pitch 30°', 'pitch 45°', 'pitch 60°', 'pitch 75°']

# Access the specific experiment
experiment_key = f"experiment{NEXP[i]}"
data = mat[experiment_key]

# Assign repetition variables (convert MATLAB structure to Python)
TIME = data.motion_tracking.TIME
PITCHavg = data.motion_tracking.PITCH_avg
VELxavg = data.motion_tracking.VEL_BODYx_filtered_avg
DIHEDavg = data.motion_tracking.DIHEDRAL_avg

ACCx = data.motion_tracking.ACC_BODYx_filtered
VELzavg = data.motion_tracking.VEL_BODYz_filtered_avg
ACCz=data.motion_tracking.ACC_BODYz_filtered
OMyavg=data.motion_tracking.OMy_filtered_avg
ALPHyavg=data.motion_tracking.ALPHy_filtered_avg
VELz=data.motion_tracking.VEL_BODYz_filtered
VELx = data.motion_tracking.VEL_BODYx_filtered

# %% Parameters
bx = 0.07220
lz = 0.02710
bz = 0.0117
c1 = 0.0114
c2 = -0.0449
f = 16.59
Iyy = 1.26e-04
m = 0.0296
g = 9.80665
lw = 0.081

# %% Interpolate existing dihedral
valid = ~np.isnan(DIHEDavg)
x_val = TIME[valid]
y_val = DIHEDavg[valid]
y_valsmooth = savgol_filter(y_val, window_length=11, polyorder=2)
f_dihed = interp1d(x_val, y_valsmooth, kind='cubic', fill_value="extrapolate")

DIHEDinterp = f_dihed(TIME)

# %% Filter the dihedral
fs = 360  # Hz
fc = 10   # Hz
order = 3

b, a = butter(order, fc / (fs / 2), btype='low')
DIHEDinterp = filtfilt(b, a, DIHEDinterp)

# %% Prepare inputs for the model
dt = np.mean(np.diff(TIME))
ld = lw * np.sin(np.radians(DIHEDinterp))
f_ld = interp1d(TIME, ld, kind='cubic', fill_value="extrapolate")

ldd = np.gradient(ld, dt)
f_ldd = interp1d(TIME, ldd, kind='cubic', fill_value="extrapolate")

# Parametrize the pitch angle
theta_smooth = savgol_filter(np.radians(PITCHavg), window_length=11, polyorder=2)
f_theta = interp1d(TIME, theta_smooth, kind='cubic', fill_value="extrapolate")

thetad_est = np.gradient(theta_smooth, dt)
f_thetad = interp1d(TIME, thetad_est, kind='cubic', fill_value="extrapolate")

f_u = interp1d(TIME, VELx, kind='cubic', fill_value="extrapolate")
f_w = interp1d(TIME, VELz, kind='cubic', fill_value="extrapolate")

# Interpolate u, w, fld, fldd from flight data
u_vals = f_u(TIME)
w_vals = f_w(TIME)
fld_vals = f_ld(TIME)
fldd_vals = f_ldd(TIME)
thetad_vals = f_thetad(TIME)
theta_vals = f_theta(TIME)

fx_model =  -np.sin(theta_vals) * g - 2*bx / m * (VELx[0] - lz * theta_vals + fldd_vals)

T = 2*(c1*f + c2)

fz_model = np.cos(theta_vals)*g - T/m - 2*bz/m * (w_vals - fld_vals*theta_vals)

moment_model = (-(- 2*bx * lz * (u_vals - lz * thetad_vals + fldd_vals)
                   + 2*bz * fld_vals * (w_vals - fld_vals * thetad_vals)
                   - T * fld_vals) / Iyy)

from scipy.optimize import least_squares

# Define least squares
# %% Plotting model behaviour
plt.figure()

plt.suptitle(f'Forces and moments model for {legend_list[i]}')
plt.subplot(3, 1, 1)
plt.plot(TIME, fx_model, linewidth=2, label='Model original', linestyle='--')
# plt.plot(TIME, fx_model_fit, linewidth=2, label='Model fitted', linestyle='--')
plt.plot(TIME, np.mean(ACCx, axis=0), label='Flight Data')
plt.title(r'$F_x$ model')
plt.ylabel(r'$\dot{u}$ ($m/s^2$)')
plt.legend()
plt.gca().set_xticklabels([])

plt.subplot(3, 1, 2)
# plt.plot(TIME, fz_model, linewidth=2, label='Model', linestyle='--')
plt.plot(TIME, np.mean(ACCz, axis=0), label='Flight Data')
plt.title(r'$F_z$ model')
plt.ylabel(r'$\dot{w}$ ($m/s^2$)')
plt.gca().set_xticklabels([])

plt.subplot(3, 1, 3)
# plt.plot(TIME, moment_model, linewidth=2, label='Model', linestyle='--')
plt.plot(TIME, np.radians(ALPHyavg), label='Flight Data')
plt.title(r'$M_y$ model')
plt.xlabel('Time (s)')
plt.ylabel(r'$\ddot{theta}$ ($rad/s^2$)')

plt.tight_layout()
plt.show()

plt.figure()

plt.suptitle('Fitted forces and moments model for {legend_list[i]}')


plt.plot(TIME, VELx[0])
plt.plot(TIME, fx_model)
plt.show()