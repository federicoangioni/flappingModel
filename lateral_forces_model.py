import numpy as np
import scipy
from scipy.signal import butter, filtfilt, tf2ss, StateSpace, TransferFunction, lsim
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# %% Define constants
g = 9.81
c_corr = 0.175
lw = 81e-3
c_v = c_w = 0.5
c1 = 0.0107  # c1 = 0.0114
c2 = -0.03925  # c2 = -0.0449
s1 = 0.2014
bat_corr = 0.8
s2 = 3.9517 * bat_corr
m = 29.85e-3
act_w0 = 40  # rad/s
act_damp = 0.634  # -

# %% Load .mat file
angle = {95: 15, 100: 30, 99: 45, 98: 60, 4: np.nan}
Nexp = 4
nman = 1
title_main = f"lateral maneuvre{angle[Nexp]}"
title_comp = f"lateral_maneuvre_components_{angle[Nexp]}"
save = False
plot = True

mat = scipy.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)

experiment_key = f"experiment{Nexp}"
data = mat[experiment_key]

# %% Extract necessary flight data
# CMD
CMDLeft = data.onboard_interpolated.CMDleft_motor_interp[nman]
CMDRight = data.onboard_interpolated.CMDright_motor_interp[nman]
CMD_dihed = np.radians(data.onboard_interpolated.CMDpitch_interp[nman] / 100 * 18)

# Time data
time = data.motion_tracking.TIME
time -= time[0]
time_onboard_rates = data.onboard.rates.TIME_onboard_rates[nman]
time_onboard_rates -= time_onboard_rates[0]
time_onboard = data.onboard.angles_commands_setpoints.TIME_onboard[nman]
time_onboard -= time_onboard[0]
time_freq = data.onboard.frequency.TIME_onboard_freq[nman]
time_freq -= time_freq[0]

# Frequency
freq_right = data.onboard.frequency.FREQright_wing[nman]

# Accelerations
u = data.motion_tracking.VEL_BODYx_filtered[nman]
v = data.motion_tracking.VEL_BODYy_filtered[nman]
w = data.motion_tracking.VEL_BODYz_filtered[nman]
vdot = data.motion_tracking.DVEL_BODYy_filtered[nman]
wdot = data.motion_tracking.DVEL_BODYz_filtered[nman]
phidd = np.radians(data.onboard.rates.ALPHx_IMU_filtered[nman])
phi_raw = np.radians(data.onboard.angles_commands_setpoints.ROLL_IMU[nman])
phid_raw = np.radians(data.onboard.rates.OMx_IMU_filtered[nman])

# Dihedral
dihedral = data.motion_tracking.DIHEDRAL[nman]

# --------------------------------------------------------------------------------
# %% Butterworth filter
fs = 1 / np.mean(np.diff(time))
fc = 20
order = 4

b, a = butter(order, fc / (fs / 2))
phidd = filtfilt(b, a, phidd)

# %% Process frequency of flapper (ok)

# Find initial frequency, static equilibrium
f0 = (m * g / 2 - c2) / c1

fL_cmd = CMDLeft * s1 + s2 - 0.5 * w + 0.5 * v
fR_cmd = CMDRight * s1 + s2 - 0.5 * w - 0.5 * v

# First order tf frequency response
A, B, C, D = tf2ss([12.56], [1, 12.56])
sys_motor = StateSpace(A, B, C, D)

_, fL_out, _ = lsim(sys_motor, U=fL_cmd, T=time, X0=f0 / C)
_, fR_out, _ = lsim(sys_motor, U=fR_cmd, T=time, X0=f0 / C)

# %% Dihedral transfer function
numerator = [act_w0**2]
denominator = [1, 2 * act_damp * act_w0, act_w0**2]

system = TransferFunction(numerator, denominator)
t_dih, y_dih, _ = lsim(system, U=CMD_dihed, T=time)

# correction on dihedral due to velocity
dih_corr = -c_corr * u + y_dih

# %% Force modeling

phi = interp1d(time_onboard, phi_raw, fill_value="extrapolate")(time)
phid = interp1d(time_onboard_rates, phid_raw, fill_value="extrapolate")(time)

dt = np.mean(np.diff(time))
ld = lw * np.sin(dih_corr)
ldd = np.gradient(ld, time)


# Thrust for one wing
def T(f):
    return c1 * f + c2


fy = np.sin(phi) * g + phi * w + 0.014 / m * (fL_out - fR_out) * (v)
fz =  - (T(fL_out) + T(fR_out)) / m + np.cos(phi) * g - 0.01 * (fL_out + fR_out) * w

# %% Plotting forces estimation
if plot:
    fig, axs = plt.subplots(6, 1, figsize=(8, 6))

    axs[0].plot(time, vdot)
    axs[0].plot(time, fy, linestyle="--", color="black")
    axs[0].set_ylabel(r"$\dot{v}$ [m/$s^2$]")
    axs[0].set_xlim(0.5, 3.5)
    axs[0].set_ylim(-20, 20)
    axs[0].spines["top"].set_visible(False)
    axs[0].spines["right"].set_visible(False)
    axs[0].set_xticklabels([])

    axs[1].plot(time, wdot)
    axs[1].plot(time, fz, linestyle="--", color="black")
    axs[1].set_xlim(0.5, 3.5)
    axs[1].set_ylim(-10, 35)
    axs[1].set_ylabel(r"$\dot{w}$ [m/$s^2$]")
    axs[1].spines["top"].set_visible(False)
    axs[1].spines["right"].set_visible(False)
    axs[1].set_xticklabels([])

    axs[2].plot(time_onboard_rates, phidd)
    axs[2].set_ylabel(r"$\ddot{\phi}$ [rad/s$^2$]")
    axs[2].set_xlim(0.5, 3.5)
    # axs[2].set_ylim(-100, 100)
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

    axs[4].plot(time_freq, freq_right, linewidth=1.0, label=r"Flight data $f_R$")
    axs[4].plot(time, fR_out, linestyle="--", color="red", label=r"Simulation $f_R$")
    axs[4].plot(time, fL_out, linestyle="--", color="blue", label=r"Simulation $f_L$")
    axs[4].set_ylabel(r"$ff$ [Hz]")
    axs[4].set_xlim(0.5, 3.5)
    axs[3].spines["top"].set_visible(False)
    axs[3].spines["right"].set_visible(False)
    axs[3].set_xticklabels([])
    axs[4].legend(fontsize=8)

    axs[5].plot(time, CMDRight, color="darkkhaki", linestyle="--", label="Setpoint")
    axs[5].set_ylabel(r"CMD thrust R [%]")
    axs[5].set_xlabel(r"Time [s]")
    axs[5].set_xlim(0.5, 3.5)
    axs[5].spines["top"].set_visible(False)
    axs[5].spines["right"].set_visible(False)
    axs[5].set_xlabel(r"Time [s]")
    axs[5].legend(fontsize=8)

    fig.align_ylabels(axs[:])
    plt.tight_layout()

    plt.suptitle("Using flight data to simulate accelerations")
    if save:
        plt.savefig(f"results/{title_main}.png")

    # %% Plotting components for forces estimation

    fig, axs = plt.subplots(3, 2, figsize=(12, 4))

    axs[0, 0].plot(time, v, linewidth=1.0, label="Flight data")
    axs[0, 0].set_ylabel(r"v [m/s]")
    axs[0, 0].set_xlim(0.5, 3.5)
    axs[0, 0].spines["top"].set_visible(False)
    axs[0, 0].spines["right"].set_visible(False)
    axs[0, 0].set_xticklabels([])

    axs[0, 1].plot(time, w, linewidth=1.0, label="Flight data")
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

    axs[2, 0].plot(time, phi, linewidth=1.0, label="Flight data")
    axs[2, 0].set_ylabel(r"$\phi$ [rad]")
    axs[2, 0].set_xlim(0.5, 3.5)
    axs[2, 0].set_xlabel(r"Time [s]")
    axs[2, 0].spines["top"].set_visible(False)
    axs[2, 0].spines["right"].set_visible(False)

    axs[2, 1].plot(time, phid, linewidth=1.0, label="Flight data")
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
