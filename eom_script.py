import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt, savgol_filter
from scipy.integrate import solve_ivp

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
VELzavg = data.motion_tracking.VEL_BODYz_filtered_avg
ACCz=data.motion_tracking.ACC_BODYz_filtered
OMyavg=data.motion_tracking.OMy_filtered_avg

ALPHyavg=data.motion_tracking.ALPHy_filtered_avg

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

f_u = interp1d(TIME, VELxavg, kind='cubic', fill_value="extrapolate")
f_w = interp1d(TIME, VELzavg, kind='cubic', fill_value="extrapolate")


# %% Solve ODE using Matej's parameters
bx = 0.07220
lz = 0.02710
bz = 0.0117
c1 = 0.0114
c2 = -0.0449
f = 16.49
Iyy = 1.26e-04

initial_conditions = [0.178, 0, 0, 0] # [u0, w0, theta0, thetad0]

def ud_ode(t, u):
    fldd = float(f_ldd(t))
    return -np.sin(f_theta(t)) * g - 2 * bx / m * (u - lz * f_thetad(t) + fldd)

T = 2*(c1*f + c2)

def wd_ode(t, w):
    return np.cos(f_theta(t))*g - T/m - 1.8*bz/m * (w - f_ld(t)*f_thetad(t))


u0 = 0.178
w0 = 0

# Solve ode
sol_ud = solve_ivp(ud_ode, [TIME[0], TIME[-1]], [u0], t_eval=TIME, method='RK45')
u_sol = sol_ud.y[0]

ud = np.array([ud_ode(t, [u]) for t, u in zip(TIME, u_sol)])

sol_wd = solve_ivp(wd_ode, [TIME[0], TIME[-1]], [w0], t_eval=TIME, method='RK45')
w_sol = sol_wd.y[0]

wd = np.array([wd_ode(t, [w]) for t, w in zip(TIME, w_sol)])

# %% Plotting

plt.figure()

plt.subplot(4,2,1)
plt.plot(TIME, DIHEDinterp, linewidth=2)
plt.title('Interpolated and Filtered DIHED(t)')
plt.xlabel('Time (s)')
plt.ylabel('DIHED (deg)')

plt.subplot(4,2,2)
plt.plot(TIME, u_sol, linewidth=2, label='Model')
plt.plot(TIME, VELxavg, label='Flight Data')
plt.title('Solution of the ODE')
plt.xlabel('Time (s)')
plt.ylabel('u(t)')
plt.legend()

plt.subplot(4,2,3)
plt.plot(TIME, ud, label='Model')
plt.plot(TIME, np.mean(ACCx, axis=0), label='Flight Data')
plt.title('Acceleration Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.subplot(4,2,4)
plt.plot(TIME, w_sol, label='Model')
plt.plot(TIME, VELzavg, label= 'Flight data')
plt.title('z velocity Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.subplot(4,2,5)
plt.plot(TIME, wd, label='Model')
plt.plot(TIME, np.mean(ACCz, axis=0), label= 'Flight data')
plt.title('z velocity Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()

plt.tight_layout()
plt.show()


# %% GPT trying to figure out
# Define Butterworth filter parameters
fs = 360  # Sampling frequency (Hz)
fc = 10   # Cutoff frequency (Hz)
order = 3

# Create the filter
b, a = butter(order, fc / (fs / 2), btype='low')

# Apply the filter to thetad_est
thetad_filt = filtfilt(b, a, thetad_est)

# Optional: Compute smoothed thetadd
thetadd_filt = np.gradient(thetad_filt, dt)

# You can also create interpolants if needed
f_thetad = interp1d(TIME, thetad_filt, kind='cubic', fill_value="extrapolate")
f_thetadd = interp1d(TIME, thetadd_filt, kind='cubic', fill_value="extrapolate")

# Compute numerical thetadd from smoothed flight data
thetadd_num = np.gradient(thetad_est, dt)

# Interpolate u, w, fld, fldd from flight data
u_vals = f_u(TIME)
w_vals = f_w(TIME)
fld_vals = f_ld(TIME)
fldd_vals = f_ldd(TIME)
thetad_vals = f_thetad(TIME)
bx = 0.16440
bz = 0.00100
lz = 0.01000
# Compute predicted thetadd from the model
thetadd_model = (-(- 2*bx * lz * (u_vals - lz * thetad_vals + fldd_vals)
                  + 2*bz * fld_vals * (w_vals - fld_vals * thetad_vals)
                  - T * fld_vals) / Iyy)

# Plot comparison
plt.figure(figsize=(10, 5))
plt.plot(TIME, thetadd_filt, label='thetadd from flight data (numerical)')
plt.plot(TIME, thetadd_model, label='thetadd from model', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel('thetadd [rad/s²]')
plt.title('Comparison of thetadd from Data vs. Model')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


term1 = -2 * bx * lz * (u_vals - lz * thetad_vals + fldd_vals)/Iyy
term2 = 1.8 * bz * fld_vals * (w_vals - fld_vals * thetad_vals)/Iyy
term3 = -T * fld_vals/Iyy

plt.plot(TIME, term1, label='Aerodynamic moment (u)')
plt.plot(TIME, term2, label='Aerodynamic moment (w)')
plt.plot(TIME, term3, label='Thrust moment')
plt.legend()
plt.show()



# from scipy.optimize import least_squares

# # Define your objective function
# def objective(params):
#     bx_, bz_, lz_ = params

#     thetadd_model = []
#     for t in TIME:
#         fld = float(f_ld(t))
#         fldd = float(f_ldd(t))
#         u = float(f_u(t))
#         w = float(f_w(t))
#         theta_val = float(f_theta(t))
#         thetad_val = float(f_thetad(t))
        
#         dthetaddt = (
#             - (-2 * bx_ * lz_ * (u - lz_ * thetad_val + fldd) 
#                + 1.8 * bz_ * fld * (w - fld * thetad_val) 
#                - T * fld) / Iyy
#         )
#         thetadd_model.append(dthetaddt)

#     thetadd_model = np.array(thetadd_model)
    
#     # Error between model and measured thetadd
#     error = thetadd_model - thetadd_num  # you already have this from your numerical deriv
#     return error

# # Initial parameters:    [bx,     bz,     lz,     T,      Iyy     ]
# x0 = np.array([0.0722, 0.0117, 0.0271])
# bounds = (
#     [0.01, 0.001, 0.01],  # lower
#     [0.2,  0.05,  0.05]   # upper
# )

# # res = least_squares(objective, x0, bounds=bounds)

# # # Best-fit parameters
# # bx_fit, bz_fit, lz_fit = res.x
# # print("Optimized parameters:")
# # print(f"bx = {bx_fit:.5f}, bz = {bz_fit:.5f}, lz = {lz_fit:.5f}")
