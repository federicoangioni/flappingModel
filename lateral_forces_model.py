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
lw = 81e-3

# %% Load .mat file
# Sideways data: 95, 100, 99, 98 for roll angle of: (15, 30, 45, 60)
angle = {95: 15, 100: 30, 99: 45, 98: 60}
Nexp = 95
nman = 0
title_main = f"lateral maneuvre{angle[Nexp]}"
title_comp = f"lateral_maneuvre_components_{angle[Nexp]}"
save = False

mat = scipy.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)

experiment_key = f"experiment{Nexp}"
data = mat[experiment_key]

# %% Extract necessary flight data

# Frequency, both onboard and interpolated + setpoints
ff = data.onboard_interpolated.FREQright_wing_interp[nman]
freq_right = data.onboard.frequency.FREQright_wing[nman]
time_freq = data.onboard.frequency.TIME_onboard_freq[nman]
CMDRight = data.onboard_interpolated.CMDthrottle_interp[nman] / 4.1

# Time data
time = data.motion_tracking.TIME
time -= time[0]
time_onboard_rates = data.onboard.rates.TIME_onboard_rates[nman]
time_onboard_rates -= time_onboard_rates[0]
time_onboard = data.onboard.angles_commands_setpoints.TIME_onboard[nman]
time_onboard -= time_onboard[0]

# Frequency, both onboard and interpolated + setpoints
ff = data.onboard_interpolated.FREQright_wing_interp[nman]
freq_right = data.onboard.frequency.FREQright_wing[nman]
time_freq = data.onboard.frequency.TIME_onboard_freq[nman]
CMDRight = data.onboard_interpolated.CMDthrottle_interp[nman] / 4.1

# Accelerations
accy = data.motion_tracking.DVEL_BODYy_filtered[nman]
accz = data.motion_tracking.DVEL_BODYz_filtered[nman]
vely = data.motion_tracking.VEL_BODYy_filtered[nman]
velz = data.motion_tracking.VEL_BODYz_filtered[nman]
velx = data.motion_tracking.VEL_BODYx_filtered[nman]
phidd_opti = np.radians(data.motion_tracking.ALPHx[nman])
phidd_onboard = data.onboard.rates.ALPHx_IMU_filtered[nman]

# Model direction specific
roll_raw = np.radians(data.onboard.angles_commands_setpoints.ROLL_IMU[nman])
roll = interp1d(time_onboard, roll_raw, fill_value="extrapolate")(time)
omx_raw = np.radians(data.onboard.rates.OMx_IMU_filtered[nman])
omx = interp1d(time_onboard_rates, omx_raw, fill_value="extrapolate")(time)

# Dihedral data
# Dihedral commands and response
dihedral = data.motion_tracking.DIHEDRAL[nman]
CMD_dihed = np.radians(data.onboard_interpolated.CMDpitch_interp[nman] / 100 * 18)

# %% Butterworth filter
fs = 1 / np.mean(np.diff(time))
fc = 5
order = 4

b, a = butter(order, fc / (fs / 2))
accz = filtfilt(b, a, accz)
accy = filtfilt(b, a, accy)
phidd_onboard = filtfilt(b, a, phidd_onboard)
phidd_opti = filtfilt(b, a, phidd_opti)

# %% Frequency transfer function

tau = 0.0796
K = 1.0
numerator = [K]
denominator = [tau, 1]
system = TransferFunction(numerator, denominator)
t_ff, y_ff, _ = lsim(system, U=CMDRight, T=time)

# %% Dihedral transfer function

act_w0 = 40  # rad/s
act_damp = 0.634  # -
numerator = [act_w0**2]
denominator = [1, 2 * act_damp * act_w0, act_w0**2]

system = TransferFunction(numerator, denominator)
t_dih, y_dih, _ = lsim(system, U=CMD_dihed, T=time)

# correction on dihedral due to velocity
dih_corr = -c_corr * velx + y_dih

# %% Force modeling
dt = np.mean(np.diff(time))
ld = lw * np.sin(dih_corr)
ldd = np.gradient(ld, time)

def T(f):
    return 2 * (c1 * f + c2)


# %% Plotting forces estimation

fig, axs = plt.subplots(5, 1, figsize=(8, 6))

axs[0].plot(time, accy)
axs[0].set_ylabel(r"$\dot{v}$ [m/$s^2$]")
axs[0].set_xlim(0.5, 3.5)
axs[0].set_ylim(-20, 20)
axs[0].spines["top"].set_visible(False)
axs[0].spines["right"].set_visible(False)
axs[0].set_xticklabels([])

axs[1].plot(time, accz)
axs[1].set_xlim(0.5, 3.5)
axs[1].set_ylim(-10, 35)
axs[1].set_ylabel(r"$\dot{w}$ [m/$s^2$]")
axs[1].spines["top"].set_visible(False)
axs[1].spines["right"].set_visible(False)
axs[1].set_xticklabels([])

axs[2].plot(time, phidd_opti)
axs[2].set_ylabel(r"$\ddot{\phi}$ [rad/s$^2$]")
axs[2].set_xlim(0.5, 3.5)
axs[2].set_ylim(-100, 100)
axs[2].spines["top"].set_visible(False)
axs[2].spines["right"].set_visible(False)
axs[2].set_xticklabels([])

axs[3].plot(time, dihedral, linewidth=1.0, label="Flight data")
axs[3].plot(
    t_dih, np.degrees(dih_corr), linestyle="--", color="black", label="Simulation"
)
axs[3].plot(
    time, np.degrees(CMD_dihed), linestyle="--", color="darkkhaki", label="Setpoint"
)
axs[3].set_ylabel(r"$\gamma_2 [deg]$")
axs[3].set_xlim(0.5, 3.5)
axs[3].spines["top"].set_visible(False)
axs[3].spines["right"].set_visible(False)
axs[3].set_xticklabels([])

axs[4].plot(time, ff, linewidth=1.0, label="Flight data")
axs[4].plot(t_ff, y_ff, linestyle="--", color="black", label="Simulation")
axs[4].plot(time, CMDRight, color="darkkhaki", linestyle="--", label="Setpoint")
axs[4].set_ylabel(r"f [Hz]")
axs[4].set_xlabel(r"Time [s]")
axs[4].set_xlim(0.5, 3.5)
axs[4].set_ylim(5.0, 27)
axs[4].spines["top"].set_visible(False)
axs[4].spines["right"].set_visible(False)
axs[4].legend(loc="upper left", bbox_to_anchor=(0.2, 0.8), fontsize=8)

fig.align_ylabels(axs[:])
plt.tight_layout()

plt.suptitle("Using flight data to simulate accelerations")
if save:
    plt.savefig(f"results/{title_main}.png")

# %% Plotting components for forces estimation

fig, axs = plt.subplots(3, 2, figsize=(12, 4))

axs[0, 0].plot(time, vely, linewidth=1.0, label="Flight data")
axs[0, 0].set_ylabel(r"v [m/s]")
axs[0, 0].set_xlim(0.5, 3.5)
axs[0, 0].spines["top"].set_visible(False)
axs[0, 0].spines["right"].set_visible(False)
axs[0, 0].set_xticklabels([])

axs[0, 1].plot(time, velz, linewidth=1.0, label="Flight data")
axs[0, 1].set_ylabel(r"w [m/s]")
axs[0, 1].set_xlim(0.5, 3.5)
axs[0, 1].spines["top"].set_visible(False)
axs[0, 1].spines["right"].set_visible(False)
axs[0, 1].set_xticklabels([])

axs[1, 0].plot(time, ld, linewidth=1.0, label="Flight data")
axs[1, 0].set_ylabel(r"$l_d$ [m]")
axs[1, 0].set_xlim(0.5, 3.5)
axs[1, 0].spines["top"].set_visible(False)
axs[1, 0].spines["right"].set_visible(False)
axs[1, 0].set_xticklabels([])

axs[1, 1].plot(time, ldd, linewidth=1.0, label="Flight data")
axs[1, 1].set_ylabel(r"$l_{dd}$ [m/s]")
axs[1, 1].set_xlim(0.5, 3.5)
axs[1, 1].spines["top"].set_visible(False)
axs[1, 1].spines["right"].set_visible(False)
axs[1, 1].set_xticklabels([])

axs[2, 0].plot(time, roll, linewidth=1.0, label="Flight data")
axs[2, 0].set_ylabel(r"$\phi$ [rad]")
axs[2, 0].set_xlim(0.5, 3.5)
axs[2, 0].set_xlabel(r"Time [s]")
axs[2, 0].spines["top"].set_visible(False)
axs[2, 0].spines["right"].set_visible(False)

axs[2, 1].plot(time, omx, linewidth=1.0, label="Flight data")
axs[2, 1].set_ylabel(r"$\dot{\phi}$ [rad/s]")
axs[2, 1].set_xlabel(r"Time [s]")
axs[2, 1].set_xlim(0.5, 3.5)
axs[2, 1].spines["top"].set_visible(False)
axs[2, 1].spines["right"].set_visible(False)
axs[2, 1].legend(fontsize=8)

fig.align_ylabels(axs[:])
plt.suptitle("Dynamic components for lateral model")

if save:
    plt.savefig(f"results/{title_comp}.png")

plt.show()
