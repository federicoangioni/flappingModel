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
c1 = 0.0114
c2 = -0.0449


def load_data(exps):
    """
    nexp: dict
    """

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
            CMD_dihed = np.radians(
                temp_data.onboard_interpolated.CMDpitch_interp[run] / 100 * 18
            )
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
                "dih_corr": dih_corr,
                "t_dih": t_dih,
                "CMD_dihed": CMD_dihed,
                "t_ff": t_ff,
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

def T(f):
    return (c1 * f + c2)


def udot_lst_longitudinal(data, v=False):
    # Extract the needed data
    time = data["time"]
    pitch = data["pitch"]
    y_ff = data["y_ff"]
    velx = data["velx"]
    dih_corr = data["dih_corr"]
    omy = data["omy"]
    velz = data["velz"]
    accx = data["accx"]

    ld = np.sin(dih_corr)
    ldd = np.gradient(ld, time)

    b = accx + omy * velz + np.sin(pitch) * g

    a1 = y_ff / m * (lz * omy - velx)
    a2 = -y_ff * ldd / m
    A = np.array([a1, a2]).T
    [k1_x, k2_x], resid, rank, s = np.linalg.lstsq(A, b)

    if v:
        print("------------------------------------------")
        print("Starting regression, using np.linalg.lstsq")
        print("------------------Done--------------------")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("------------------Results-----------------")
        print("Visualising the results:                  ")
        print(f"bx  = {np.round(k1_x, 10)}, lw*bx = {np.round(k2_x, 10)} ")
        r2 = 1 - resid / sum((b - b.mean()) ** 2)

        print(f"Value of R^2 = {r2}")

    return [k1_x, k2_x]


def wdot_lst_longitudinal(data, v=False):
    # Extract the needed data
    pitch = data["pitch"]
    y_ff = data["y_ff"]
    velx = data["velx"]
    dih_corr = data["dih_corr"]
    omy = data["omy"]
    velz = data["velz"]
    accz = data["accz"]

    ld = np.sin(dih_corr)

    b = accz - np.cos(pitch) * g - omy * velx + 2 * T(y_ff) / m

    a1 = -y_ff * velz / m
    a2 = y_ff * ld * omy / m
    A = np.array([a1, a2]).T

    [k1_z, k2_z], resid, rank, s = np.linalg.lstsq(A, b)

    if v:
        print("------------------------------------------")
        print("Starting regression, using np.linalg.lstsq")
        print("------------------Done--------------------")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("                                          ")
        print("------------------Results-----------------")
        print("Visualising the results:                  ")
        print(f"bz  = {np.round(k1_z, 10)}, lw * bz = {np.round(k2_z, 10)}")
        r2 = 1 - resid / sum((b - b.mean()) ** 2)

        print(f"Value of R^2 = {r2}")

    return [k1_z, k2_z]


experiments = {102: range(0, 7), 101:[0], 103:[0], 94:[0], 104:[0]}

# experiments = [102, 101, 103, 94, 104]
data = load_data(experiments)

k1, k2 = udot_lst_longitudinal(data, v= True)
k1, k2 = wdot_lst_longitudinal(data, v= True)