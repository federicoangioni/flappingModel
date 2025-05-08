import scipy as sp
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt
from scipy.signal import TransferFunction, lsim

mat = sp.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)

# Constants
c_corr = 0.175
g = 9.80665
m = 29.4e-3
lz = 27e-3

def load_data(nexp=None):
    """
    nexp: list
    """

    # Select the experiments to extract
    data = tuple(mat[f"experiment{experiment}"] for experiment in nexp)

    dictionary = {
        f"experiment{experiment}": data[i] for i, experiment in enumerate(nexp)
    }

    for index, exp in enumerate((nexp)):
        # Find the number of runs per experiment
        nruns = data[index].onboard.rates.TIME_onboard_rates.shape[0]
        temp_data = dictionary[f"experiment{exp}"]
        dictionary[f"experiment{exp}"] = []

        for run in range(nruns):
            # %% Extract necessary flight data
            # Optitrack time
            time = temp_data.motion_tracking.TIME
            time -= time[0]
            thetadd_opti = np.radians(temp_data.motion_tracking.ALPHy[run])
            thetadd_onboard = temp_data.onboard.rates.ALPHy_IMU_filtered[run]
            time_onboard_rates = temp_data.onboard.rates.TIME_onboard_rates[run]
            time_onboard_rates -= time_onboard_rates[0]
            # Frequency, both onboard and interpolated + setpoints
            ff = temp_data.onboard_interpolated.FREQright_wing_interp[run]
            freq_right = temp_data.onboard.frequency.FREQright_wing[run]
            CMDRight = temp_data.onboard_interpolated.CMDthrottle_interp[run] / 4.1

            # Accelerations
            accx = temp_data.motion_tracking.DVEL_BODYx_filtered[run]
            accz = temp_data.motion_tracking.DVEL_BODYz_filtered[run]
            velx = temp_data.motion_tracking.VEL_BODYx_filtered[run]
            velz = temp_data.motion_tracking.VEL_BODYz_filtered[run]

            time_onboard = temp_data.onboard.angles_commands_setpoints.TIME_onboard[run]
            dihedral = temp_data.motion_tracking.DIHEDRAL[run]
            CMD_dihed =np.radians(temp_data.onboard_interpolated.CMDpitch_interp[run]/100*18)
            pitch_raw = np.radians(
                temp_data.onboard.angles_commands_setpoints.PITCH_IMU[run]
            )
            pitch = interp1d(time_onboard, pitch_raw, fill_value="extrapolate")(time)
            omy_raw = np.radians(temp_data.onboard.rates.OMy_IMU_filtered[run])
            omy = interp1d(time_onboard_rates, omy_raw, fill_value="extrapolate")(time)

            # %% Process the data
            # Butterworth filter
            fs = 1 / np.mean(np.diff(time))
            fc = 5
            order = 4

            b, a = butter(order, fc / (fs / 2))
            accz = filtfilt(b, a, accz)
            accx = filtfilt(b, a, accx)
            thetadd_onboard = filtfilt(b, a, thetadd_onboard)
            thetadd_opti = filtfilt(b, a, thetadd_opti)

            # Frequency transfer function
            tau = 0.0796
            K = 1.0
            numerator = [K]
            denominator = [tau, 1]
            system = TransferFunction(numerator, denominator)
            t_ff, y_ff, _ = lsim(system, U=CMDRight, T=time)
            
            # Dihedral transfer function
            act_w0 = 40  # rad/s
            act_damp = 0.634  # -
            numerator = [act_w0**2]
            denominator = [1, 2 * act_damp * act_w0, act_w0**2]

            system = TransferFunction(numerator, denominator)
            t_dih, y_dih, _ = lsim(system, U=CMD_dihed, T=time)

            # correction on dihedral due to velocity
            dih_corr = -c_corr * velx + y_dih
            
            data_run = {
                "run": [run],
                "time": time,
                "thetadd_opti": thetadd_opti,
                "thetadd_onboard": thetadd_onboard,
                "ff": ff,
                "freq_right": freq_right,
                "CMDRight": CMDRight,
                "dihedral": dihedral,
                "accx": accx,
                "accz": accz,
                "velx": velx,
                "velz": velz,
                "pitch": pitch,
                "omy": omy,
                "y_ff": y_ff,
                "dih_corr":dih_corr,
            }

            dictionary[f"experiment{exp}"].append(data_run)
    
    collected_data = {key: [] for key in dictionary[f"experiment{nexp[0]}"][0].keys()}
    for exp in nexp:
        for run in range(len(dictionary[f"experiment{exp}"])):
            curr_data = dictionary[f"experiment{exp}"][run]
            for key in curr_data.keys():
                collected_data[key].extend(curr_data[key])

    for key in collected_data.keys():
        collected_data[key] = np.array(collected_data[key])
        
    return collected_data


def forces_xdir(data):
    # Extract the needed data
    time     = data["time"]
    pitch    = data["pitch"]
    y_ff     = data["y_ff"]
    velx     = data["velx"]
    dih_corr = data["dih_corr"]
    omy      = data["omy"]
    velz     = data["velz"]
    accx     = data["accx"]
    
    ld = np.sin(dih_corr)
    ldd = np.gradient(ld, time)
    
    b = accx + omy * velz + np.sin(pitch) * g
    
    a1 = y_ff / m * (lz * omy - velx)
    a2 = - y_ff * ldd / m
    A = np.array([a1, a2, np.ones(len(a1))])
    
    np.linalg.lstsq(A, b)
    
experiments = [104, 94]
data = load_data(experiments)

forces_xdir(data)
