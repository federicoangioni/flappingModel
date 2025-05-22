import scipy as sp
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt, tf2ss, StateSpace
from scipy.signal import TransferFunction, lsim

mat = sp.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)

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
lz = 27e-3
Ixx = 1.02e-4


def load_data(exps):

    # Select the experiments to extract
    data = tuple(mat[f"experiment{experiment}"] for experiment in exps.keys())

    dictionary = {
        f"experiment{experiment}": data[i] for i, experiment in enumerate(exps.keys())
    }
    
    
    for index, exp in enumerate((exps.keys())):
        # Find the number of runs per experiment
        temp_data = dictionary[f"experiment{exp}"]
        dictionary[f"experiment{exp}"] = []

        for run in list(list(exps.values())[index]):
            # %% Extract necessary flight temp_data
            # CMD
            CMDLeft = temp_data.onboard_interpolated.CMDleft_motor_interp[exp]
            CMDRight = temp_data.onboard_interpolated.CMDright_motor_interp[exp]
            CMD_dihed = np.radians(temp_data.onboard_interpolated.CMDpitch_interp[exp] / 100 * 18)

            # Time temp_data
            time = temp_data.motion_tracking.TIME
            time -= time[0]
            time_onboard_rates = temp_data.onboard.rates.TIME_onboard_rates[exp]
            time_onboard_rates -= time_onboard_rates[0]
            time_onboard = temp_data.onboard.angles_commands_setpoints.TIME_onboard[exp]
            time_onboard -= time_onboard[0]
            time_freq = temp_data.onboard.frequency.TIME_onboard_freq[exp]
            time_freq -= time_freq[0]

            # Frequency
            freq_right = temp_data.onboard.frequency.FREQright_wing[exp]

            # Accelerations
            u = temp_data.motion_tracking.VEL_BODYx_filtered[exp]
            v = temp_data.motion_tracking.VEL_BODYy_filtered[exp]
            w = temp_data.motion_tracking.VEL_BODYz_filtered[exp]
            vdot = temp_data.motion_tracking.DVEL_BODYy_filtered[exp]
            wdot = temp_data.motion_tracking.DVEL_BODYz_filtered[exp]
            phidd = np.radians(temp_data.onboard.rates.ALPHx_IMU_filtered[exp])
            phi_raw = np.radians(temp_data.onboard.angles_commands_setpoints.ROLL_IMU[exp])
            phid_raw = np.radians(temp_data.onboard.rates.OMx_IMU_filtered[exp])

            # Dihedral
            dihedral = temp_data.motion_tracking.DIHEDRAL[exp]
            
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
            
            data_run = {
                "run": [run],
                "time": time,
                "vdot": vdot,
                "wdot": wdot,
                "fL": fL_out,
                "fR": fR_out,
                "phid": phid,
                "dihedral": dihedral,
            }

            dictionary[f"experiment{exp}"].append(data_run)
            
            
    collected_data = {key: [] for key in dictionary[f"experiment{list(exps.keys())[0]}"][0].keys()}
    for exp in exps.keys():
        for run in range(len(dictionary[f"experiment{exp}"])):
            curr_data = dictionary[f"experiment{exp}"][run]
            for key in curr_data.keys():
                collected_data[key].extend(curr_data[key])

    for key in collected_data.keys():
        collected_data[key] = np.array(collected_data[key])

    return collected_data
            
            

# Thrust for one wing
def T(f):
    return c1 * f + c2