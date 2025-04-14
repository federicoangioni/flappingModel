import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt, savgol_filter
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

# %% Load the data
i = 1  # Index for first experiment (Python uses 0-based indexing)

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
VELz = data.motion_tracking.VEL_BODYz_filtered_avg
ACCz=data.motion_tracking.ACC_BODYz_filtered
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

# %% Differential equation parameters
m = 0.0296
g = 9.80665
lw = 0.081

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

# %% Solve ODE using Matej's parameters
bx = 0.07220
lz = 0.02710
bz = 0.0117
c1 = 0.0114
c2 = -0.0449
f = 16.48
Iyy = 1.260000000000000e-04

initial_conditions = [0.178, 0, 0, 0] # [u0, w0, theta0, thetad0]

def ud_ode(t, u):
    fldd = float(f_ldd(t))
    return -np.sin(f_theta(t)) * g - 2 * bx / m * (u - lz * f_thetad(t) + fldd)

T = 2*(c1*f + c2)

def wd_ode(t, w):
    return np.cos(f_theta(t))*g - T/m - 1.8*bz/m * (w - f_ld(t)*f_thetad(t))


def EoM(t, vars):
    u, w, theta, thetad = vars
    
    fld = float(f_ld(t))
    fldd = float(f_ldd(t))
    
    # Using precalculated values
    dudt = -np.sin(f_theta(t)) * g - 2 * bx / m * (u - lz * f_thetad(t) + fldd)
    dwdt = np.cos(f_theta(t)) * g - T / m - 1.8 * bz / m * (w - fld * f_thetad(t))
    
    # dudt = -np.sin(theta) * g - 2 * bx / m * (u - lz * thetad + fldd)
    # dwdt = np.cos(theta) * g - T / m - 1.8 * bz / m * (w - fld * thetad)
    
    dthetadt = thetad
    dthetaddt = (-2*bx * lz * (u - lz * thetad + fldd) + 1.8 * bz * fld * (w - fld * thetad) - T * fld)/Iyy
    
    return [dudt, dwdt, dthetadt, dthetaddt]

u0 = 0
w0 = 0
solution = solve_ivp(EoM, [TIME[0], TIME[-1]], initial_conditions, t_eval=TIME, method='RK45')

u_all = solution.y[0]

# Solve ode
sol_ud = solve_ivp(ud_ode, [TIME[0], TIME[-1]], [u0], t_eval=TIME, method='RK45')
u_sol = sol_ud.y[0]

ud = np.array([ud_ode(t, [u]) for t, u in zip(TIME, u_sol)])

sol_wd = solve_ivp(wd_ode, [TIME[0], TIME[-1]], [w0], t_eval=TIME, method='RK45')
w_sol = sol_wd.y[0]

wd = np.array([wd_ode(t, [w]) for t, w in zip(TIME, w_sol)])

# %% Plotting

plt.figure(figsize=(12, 10))

plt.subplot(5,1,1)
plt.plot(TIME, DIHEDinterp, linewidth=2)
plt.title('Interpolated and Filtered DIHED(t)')
plt.xlabel('Time (s)')
plt.ylabel('DIHED (deg)')

plt.subplot(5,1,2)
plt.plot(TIME, u_sol, linewidth=2, label='Model')
plt.plot(TIME, u_all, label='Solving all together')
plt.plot(TIME, VELxavg, label='Flight Data')
plt.title('Solution of the ODE')
plt.xlabel('Time (s)')
plt.ylabel('u(t)')
plt.legend()

plt.subplot(5,1,3)
plt.plot(TIME, ud, label='Model')
plt.plot(TIME, np.mean(ACCx, axis=0), label='Flight Data')
plt.title('Acceleration Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.subplot(5,1,4)
plt.plot(TIME, w_sol, label='Model')
plt.plot(TIME, VELz, label= 'Flight data')
plt.title('z velocity Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.subplot(5,1,5)
plt.plot(TIME, wd, label='Model')
plt.plot(TIME, np.mean(ACCz, axis=0), label= 'Flight data')
plt.title('z velocity Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.tight_layout()
plt.show()
