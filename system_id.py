import scipy as sp
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt
from scipy.signal import TransferFunction, lsim

mat = sp.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)


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

        for run in range(0, 1):
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
            time_freq = temp_data.onboard.frequency.TIME_onboard_freq[run]
            CMDRight = temp_data.onboard_interpolated.CMDthrottle_interp[run] / 4.1

            # Accelerations
            accx = temp_data.motion_tracking.DVEL_BODYx_filtered[run]
            accz = temp_data.motion_tracking.DVEL_BODYz_filtered[run]
            velx = temp_data.motion_tracking.VEL_BODYx_filtered[run]
            velz = temp_data.motion_tracking.VEL_BODYz_filtered[run]

            time_onboard = temp_data.onboard.angles_commands_setpoints.TIME_onboard[run]
            dihedral = temp_data.motion_tracking.DIHEDRAL[run]
            pitch_raw = np.radians(
                temp_data.onboard.angles_commands_setpoints.PITCH_IMU[run]
            )
            pitch = interp1d(time_onboard, pitch_raw, fill_value="extrapolate")(time)
            omy_raw = np.radians(temp_data.onboard.rates.OMy_IMU_filtered[run])
            omy = interp1d(time_onboard_rates, omy_raw, fill_value="extrapolate")(time)

            # %% Process the data
            # Butterworth filter
            fs = 1/np.mean(np.diff(time))
            fc = 5
            order = 4

            b, a = butter(order, fc / (fs/2))
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

            data_run = {
                "run": run,
                "time": time,
                "thetadd_opti": thetadd_opti,
                "thetadd_onboard": thetadd_onboard,
                "time_onboard_rates": time_onboard_rates,
                "ff": ff,
                "freq_right": freq_right,
                "time_freq": time_freq,
                "CMDRight": CMDRight,
                "accx": accx,
                "accz": accz,
                "velx": velx,
                "velz": velz,
                "dihedral": dihedral,
                "pitch": pitch,
                "omy": omy,
                "y_ff": y_ff,
            }

            dictionary[f"experiment{exp}"].append(data_run)

    return dictionary


def fit_forces(data, exps):
    velx = []
    pitch = []
    for exp in exps:        
        for run in range(len(data[f"experiment{exp}"])):
            curr_data = data[f"experiment{exp}"][run]
            
            curr_velx = curr_data["velx"]
            curr_pitch = curr_data["pitch"]
            
            velx.extend(curr_velx)
            pitch.extend(curr_pitch)
    
    return print(velx, pitch)
        

experiments = [94]
data = load_data(experiments)
fit_forces(data, experiments)
