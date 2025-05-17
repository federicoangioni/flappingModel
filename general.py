from scipy.signal import butter, filtfilt, tf2ss, StateSpace, TransferFunction, lsim

class Flapper:
    def __init__(self):
        self.m = 0
        
    def flapping_frequency(self, CMD):
        
        f0 = (m * g / 2 - c2) / c1

        f_cmd = CMD * s1 + s2 - 0.5 * w + 0.5 * v

        # First order tf frequency response
        A, B, C, D = tf2ss([12.56], [1, 12.56])
        sys_motor = StateSpace(A, B, C, D)

        _, fL_out, _ = lsim(sys_motor, U=f_cmd, T=time, X0=f0 / C)