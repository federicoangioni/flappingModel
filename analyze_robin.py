# FUNCTIONS FOR LOADING AND ANALYZING FLIGHT DATA
import csv
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from quadcopter_animation import animation

# rotatation matrix to convert from body to world frame
def Rmat(phi, theta, psi):
    Rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],[0, 1, 0],[-np.sin(theta), 0, np.cos(theta)]])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],[np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

# quaternion functions https://personal.utdallas.edu/~sxb027100/dock/quaternion.html
def quadMult(ql, qr):
    qli, qlx, qly, qlz = ql
    qri, qrx, qry, qrz = qr
    res = np.array([
        qli*qri - qlx*qrx - qly*qry - qlz*qrz,
        qlx*qri + qli*qrx + qly*qrz - qlz*qry,
        qli*qry - qlx*qrz + qly*qri + qlz*qrx,
        qli*qrz + qlx*qry - qly*qrx + qlz*qri
    ])
    return res

def quadRotate(q, v):
    qi, qx, qy, qz = q
    vx, vy, vz = v
    qv = np.array([0, vx, vy, vz])
    q_qv = quadMult(q, qv)
    qv = quadMult(q_qv, np.array([qi, -qx, -qy, -qz]))
    return qv[1:]

def quat_of_axang(ax, ang):
    ang2 = ang/2
    cang2 = np.cos(ang2)
    sang2 = np.sin(ang2)
    q = np.array([
        cang2,
        ax[0]*sang2,
        ax[1]*sang2,
        ax[2]*sang2
    ])
    return q

# testing new att setpoint calculation
def att_thrust_sp_from_acc(quat_NED, acc_sp_NED):
    # we need to rotate the drone such that the z-up axis is aligned with accSpNed-g
    # let v1 = accSpNed-g, v2 = z-up (both in NED frame)
    # then we need to rotate v2 to v1
    # the rotation axis is the cross product of v1 and v2
    # the rotation angle is the angle between v1 and v2
    
    v1 = acc_sp_NED - np.array([0, 0, 9.81])
    v2_body = np.array([0, 0, -1])
    # get v2 in NED frame (rotate v2_body by quat_NED)
    v2 = quadRotate(quat_NED, v2_body)
    # rotation axis
    axis = np.cross(v2, v1)
    axis = axis/np.linalg.norm(axis)
    angle = np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    # convert axis and angle to quaternion
    attErrNed = quat_of_axang(axis, angle)
    # attitude setpoint
    att_sp_NED = quadMult(attErrNed, quat_NED)
    
    # thrust setpoint (acc_sp projected onto z-up axis)
    T_sp = -np.dot(v1, v2)
    
    # thrust setpoint 'direct'
    T_sp = -np.linalg.norm(v1)
    
    # EXTRA: get rate sp
    # we get the axis in body frame:
    quat_NED_inv = np.array([quat_NED[0], -quat_NED[1], -quat_NED[2], -quat_NED[3]])
    axis_body = quadRotate(quat_NED_inv, axis)
    
    # and scale the axis by 'angle' to get the att_error_axis
    att_error_axis = axis_body*angle
    
    # multiply by a gain to get the rate setpoint
    rate_sp = np.array([10, 10, 5]) * att_error_axis    
    
    return att_sp_NED, T_sp, rate_sp


def quat_to_euler(q):
    qw, qx, qy, qz = q
    phi = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    theta = np.arcsin(2*(qw*qy - qz*qx))
    psi = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return phi, theta, psi
    

def load_flight_data(file_name, new_format=True):
    print('Loading', file_name)
    with open(file_name) as file:
        reader = csv.reader(file)
        data = list(reader)
        
        # separate header and data
        header = [d for d in data if len(d)==2]
        header = {d[0]: d[1] for d in header}
        
        data = [d for d in data if len(d)>2]
        keys = data[0]
        data = data[1:]
        data = np.array([[float(d) if d else np.nan for d in row] for row in data]) # set empty strings to nan
        data = dict(zip(keys,data.T))
        
        # renaming keys
        data['t'] = data['time']*1e-6 # us to s
        data['t'] -= data['t'][0] # start at 0
        
        # print logging frequency
        print('Logging frequency:', 1/np.mean(np.diff(data['t'])))

        # STATE ESTIMATE
        if 'pos[0]' in data:
            data['x'] = data.pop('pos[0]')*1e-3 # mm to m
            data['y'] = data.pop('pos[1]')*1e-3 # mm to m
            data['z'] = data.pop('pos[2]')*1e-3 # mm to m
        if 'vel[0]' in data:
            data['vx'] = data.pop('vel[0]')*1e-2 # cm/s to m/s
            data['vy'] = data.pop('vel[1]')*1e-2 # cm/s to m/s
            data['vz'] = data.pop('vel[2]')*1e-2 # cm/s to m/s
        
        quat_scaling = ((127 << 6) - 1) # float from -1 to +1 to VB that takes up 2 bytes max
        data['qw'] = data.pop('quat[0]')/quat_scaling
        data['qx'] = data.pop('quat[1]')/quat_scaling
        data['qy'] = data.pop('quat[2]')/quat_scaling
        data['qz'] = data.pop('quat[3]')/quat_scaling
        
        # quat to eulers
        data['phi'] = np.arctan2(2*(data['qw']*data['qx'] + data['qy']*data['qz']), 1 - 2*(data['qx']**2 + data['qy']**2))
        data['theta'] = np.arcsin(2*(data['qw']*data['qy'] - data['qz']*data['qx']))
        data['psi'] = np.arctan2(2*(data['qw']*data['qz'] + data['qx']*data['qy']), 1 - 2*(data['qy']**2 + data['qz']**2))
        
        # SETPOINTS
        if 'posSp[0]' in data:
            data['x_sp'] = data.pop('posSp[0]')*1e-3 # mm to m
            data['y_sp'] = data.pop('posSp[1]')*1e-3
            data['z_sp'] = data.pop('posSp[2]')*1e-3 
            
            data['vx_sp'] = data.pop('velSp[0]')*1e-2 # cm/s to m/s
            data['vy_sp'] = data.pop('velSp[1]')*1e-2
            data['vz_sp'] = data.pop('velSp[2]')*1e-2
            
            data['awx_sp'] = data.pop('accSp[0]')*1e-2 # cm/s^2 to m/s^2
            data['awy_sp'] = data.pop('accSp[1]')*1e-2
            data['awz_sp'] = data.pop('accSp[2]')*1e-2
            
            data['qw_sp'] = data.pop('quatSp[0]')/quat_scaling
            data['qx_sp'] = data.pop('quatSp[1]')/quat_scaling
            data['qy_sp'] = data.pop('quatSp[2]')/quat_scaling
            data['qz_sp'] = data.pop('quatSp[3]')/quat_scaling
        
            # quat to eulers
            data['phi_sp'] = np.arctan2(2*(data['qw_sp']*data['qx_sp'] + data['qy_sp']*data['qz_sp']), 1 - 2*(data['qx_sp']**2 + data['qy_sp']**2))
            data['theta_sp'] = np.arcsin(2*(data['qw_sp']*data['qy_sp'] - data['qz_sp']*data['qx_sp']))
            data['psi_sp'] = np.arctan2(2*(data['qw_sp']*data['qz_sp'] + data['qx_sp']*data['qy_sp']), 1 - 2*(data['qy_sp']**2 + data['qz_sp']**2))
        
        # euler_sp = np.array([
        #     quat_to_euler([qw, qx, qy, qz]) for qw, qx, qy, qz in zip(data['qw_sp'], data['qx_sp'], data['qy_sp'], data['qz_sp'])
        # ])
        # data['phi_sp2'] = euler_sp[:, 0]
        # data['theta_sp2'] = euler_sp[:, 1]
        # data['psi_sp2'] = euler_sp[:, 2] 
        
        data['p_sp'] = data.pop('gyroSp[0]')*np.pi/180 # deg/s to rad/s
        data['q_sp'] = data.pop('gyroSp[1]')*np.pi/180
        data['r_sp'] = data.pop('gyroSp[2]')*np.pi/180
        
        # data['dp_sp'] = data.pop('alphaSp[0]')*np.pi/180 # deg/s^2 to rad/s^2
        # data['dq_sp'] = data.pop('alphaSp[1]')*np.pi/180
        # data['dr_sp'] = data.pop('alphaSp[2]')*np.pi/180
        
        data['spf_sp_x'] = data.pop('spfSp[0]')/100
        data['spf_sp_y'] = data.pop('spfSp[1]')/100
        data['spf_sp_z'] = data.pop('spfSp[2]')/100
        
        data['T_sp'] = data['spf_sp_z']
        
        # INDI
        data['alpha[0]'] = data.pop('alpha[0]')*10*np.pi/180
        data['alpha[1]'] = data.pop('alpha[1]')*10*np.pi/180
        data['alpha[2]'] = data.pop('alpha[2]')*10*np.pi/180
        
        data['omega[0]'] = data['omegaUnfiltered[0]'] # already in rad/s
        data['omega[1]'] = data['omegaUnfiltered[1]']
        data['omega[2]'] = data['omegaUnfiltered[2]']
        data['omega[3]'] = data['omegaUnfiltered[3]']
        
        data['omega_dot[0]'] = data.pop('omega_dot[0]')*100
        data['omega_dot[1]'] = data.pop('omega_dot[1]')*100
        data['omega_dot[2]'] = data.pop('omega_dot[2]')*100
        data['omega_dot[3]'] = data.pop('omega_dot[3]')*100
        
        data['u[0]'] = data['u[0]']/quat_scaling
        data['u[1]'] = data['u[1]']/quat_scaling
        data['u[2]'] = data['u[2]']/quat_scaling
        data['u[3]'] = data['u[2]']/quat_scaling
        
        
        # MOTOR CMDS
        motor_limits = header['motorOutput'].split(',')
        umin = int(motor_limits[0])
        umax = int(motor_limits[1])
        data['motor_min'] = umin
        data['motor_max'] = umax
        data['u1'] = (data['motor[0]'] - umin)/(umax - umin)
        data['u2'] = (data['motor[1]'] - umin)/(umax - umin)
        data['u3'] = (data['motor[2]'] - umin)/(umax - umin)
        data['u4'] = (data['motor[3]'] - umin)/(umax - umin)
        
        # OPTITRACK
        if 'extPos[0]' in data:
            data['x_opti'] = data.pop('extPos[0]')*1e-3 # mm to m
            data['y_opti'] = data.pop('extPos[1]')*1e-3 # mm to m
            data['z_opti'] = data.pop('extPos[2]')*1e-3 # mm to m
            
            data['vx_opti'] = data.pop('extVel[0]')*1e-2 # cm/s to m/s
            data['vy_opti'] = data.pop('extVel[1]')*1e-2 # cm/s to m/s
            data['vz_opti'] = data.pop('extVel[2]')*1e-2 # cm/s to m/s
            
            data['vel_test'] = np.sqrt(data['vx_opti']**2 + data['vy_opti']**2 + data['vz_opti']**2)
            
        if 'extAtt[0]' in data:
            data['phi_opti'] = data.pop('extAtt[0]')/1000 # mrad to rad
            data['theta_opti'] = data.pop('extAtt[1]')/1000 # mrad to rad
            data['psi_opti'] = data.pop('extAtt[2]')/1000 # mrad to rad
            
            # get optitrack body velocities
            v_body_opti = np.stack([
                Rmat(phi, theta, psi).T@[vx, vy, vz]
                for vx, vy, vz, phi, theta, psi
                in zip(data['vx_opti'],data['vy_opti'],data['vz_opti'],data['phi_opti'],data['theta_opti'],data['psi_opti'])
            ])
            data['vbx_opti'] = v_body_opti[:,0]
            data['vby_opti'] = v_body_opti[:,1]
            data['vbz_opti'] = v_body_opti[:,2]
        if 'extQuat[0]' in data:
            data['qw_opti'] = data.pop('extQuat[0]')/quat_scaling
            data['qx_opti'] = data.pop('extQuat[1]')/quat_scaling
            data['qy_opti'] = data.pop('extQuat[2]')/quat_scaling
            data['qz_opti'] = data.pop('extQuat[3]')/quat_scaling
            
            data['phi_opti'] = np.arctan2(2*(data['qw_opti']*data['qx_opti'] + data['qy_opti']*data['qz_opti']), 1 - 2*(data['qx_opti']**2 + data['qy_opti']**2))
            data['theta_opti'] = np.arcsin(2*(data['qw_opti']*data['qy_opti'] - data['qz_opti']*data['qx_opti']))
            data['psi_opti'] = np.arctan2(2*(data['qw_opti']*data['qz_opti'] + data['qx_opti']*data['qy_opti']), 1 - 2*(data['qy_opti']**2 + data['qz_opti']**2))

            # get optitrack body velocities
            v_body_opti = np.stack([
                Rmat(phi, theta, psi).T@[vx, vy, vz]
                for vx, vy, vz, phi, theta, psi
                in zip(data['vx_opti'],data['vy_opti'],data['vz_opti'],data['phi_opti'],data['theta_opti'],data['psi_opti'])
            ])
            data['vbx_opti'] = v_body_opti[:,0]
            data['vby_opti'] = v_body_opti[:,1]
            data['vbz_opti'] = v_body_opti[:,2]
        
        # IMU
        gyro_scale = np.pi/180 # deg/s to rad/s
        if bool(float(header['blackbox_high_resolution'])):
            gyro_scale /= 10
        data['p'] = data.pop('gyroADC[0]')*gyro_scale       # (from FLU to FRD)
        data['q'] =-data.pop('gyroADC[1]')*gyro_scale
        data['r'] =-data.pop('gyroADC[2]')*gyro_scale
        if new_format:
            data['q']=-data['q']
            data['r']=-data['r']
        
        acc_scale = 9.81/float(header['acc_1G'])
        data['ax'] = data.pop('accSmooth[0]')*acc_scale     # (from FLU to FRD)
        data['ay'] =-data.pop('accSmooth[1]')*acc_scale
        data['az'] =-data.pop('accSmooth[2]')*acc_scale
        if new_format:
            data['ay']=-data['ay']
            data['az']=-data['az']
        
        if 'accUnfiltered[0]' in data.keys():
            data['ax_unfiltered'] = data.pop('accUnfiltered[0]')*acc_scale     # (from FLU to FRD)
            data['ay_unfiltered'] =-data.pop('accUnfiltered[1]')*acc_scale
            data['az_unfiltered'] =-data.pop('accUnfiltered[2]')*acc_scale
            if new_format:
                data['ay_unfiltered']=-data['ay_unfiltered']
                data['az_unfiltered']=-data['az_unfiltered']
        elif 'accADCafterRpm[0]' in data.keys():
            data['ax_unfiltered'] = data.pop('accADCafterRpm[0]')*acc_scale
            data['ay_unfiltered'] =-data.pop('accADCafterRpm[1]')*acc_scale
            data['az_unfiltered'] =-data.pop('accADCafterRpm[2]')*acc_scale
            if new_format:
                data['ay_unfiltered']=-data['ay_unfiltered']
                data['az_unfiltered']=-data['az_unfiltered']
        if 'acc_modeled[0]' in data.keys():
            data['ax_modeled'] = data.pop('acc_modeled[0]')/1000 # mm to m
            data['ay_modeled'] = data.pop('acc_modeled[1]')/1000
            data['az_modeled'] = data.pop('acc_modeled[2]')/1000
        
        # filter acc
        cutoff = 8 # Hz
        sos = sp.signal.butter(2, cutoff, 'low', fs=1/np.mean(np.diff(data['t'])), output='sos')
        data['ax_filt'] = sp.signal.sosfiltfilt(sos, data['ax'])
        data['ay_filt'] = sp.signal.sosfiltfilt(sos, data['ay'])
        data['az_filt'] = sp.signal.sosfiltfilt(sos, data['az'])
        
        # EKF
        if 'ekf_pos[0]' in data:
            data['ekf_x'] = data.pop('ekf_pos[0]')*1e-3 # mm to m
            data['ekf_y'] = data.pop('ekf_pos[1]')*1e-3 # mm to m
            data['ekf_z'] = data.pop('ekf_pos[2]')*1e-3 # mm to m
            data['ekf_vx'] = data.pop('ekf_vel[0]')*1e-2 # cm/s to m/s
            data['ekf_vy'] = data.pop('ekf_vel[1]')*1e-2 # cm/s to m/s
            data['ekf_vz'] = data.pop('ekf_vel[2]')*1e-2 # cm/s to m/s
            if 'ekf_att[0]' in data:
                data['ekf_phi'] = data.pop('ekf_att[0]')/1000 # mrad to rad
                data['ekf_theta'] = data.pop('ekf_att[1]')/1000 # mrad to rad
                data['ekf_psi'] = data.pop('ekf_att[2]')/1000 # mrad to rad
            if 'ekf_quat[0]' in data:
                data['ekf_qw'] = data.pop('ekf_quat[0]')/quat_scaling
                data['ekf_qx'] = data.pop('ekf_quat[1]')/quat_scaling
                data['ekf_qy'] = data.pop('ekf_quat[2]')/quat_scaling
                data['ekf_qz'] = data.pop('ekf_quat[3]')/quat_scaling
                data['ekf_phi'] = np.arctan2(2*(data['ekf_qw']*data['ekf_qx'] + data['ekf_qy']*data['ekf_qz']), 1 - 2*(data['ekf_qx']**2 + data['ekf_qy']**2))
                data['ekf_theta'] = np.arcsin(2*(data['ekf_qw']*data['ekf_qy'] - data['ekf_qz']*data['ekf_qx']))
                data['ekf_psi'] = np.arctan2(2*(data['ekf_qw']*data['ekf_qz'] + data['ekf_qx']*data['ekf_qy']), 1 - 2*(data['ekf_qy']**2 + data['ekf_qz']**2))
            data['ekf_acc_b_x'] = data.pop('ekf_acc_b[0]')/1000 # mm/s^2 to m/s^2
            data['ekf_acc_b_y'] = data.pop('ekf_acc_b[1]')/1000 # mm/s^2 to m/s^2
            data['ekf_acc_b_z'] = data.pop('ekf_acc_b[2]')/1000 # mm/s^2 to m/s^2
            data['ekf_gyro_b_x'] = data.pop('ekf_gyro_b[0]')/1000*np.pi/180
            data['ekf_gyro_b_y'] = data.pop('ekf_gyro_b[1]')/1000*np.pi/180
            data['ekf_gyro_b_z'] = data.pop('ekf_gyro_b[2]')/1000*np.pi/180
        
            data['ax_filt_unbiased'] = data['ax_filt'] - data['ekf_acc_b_x']
            data['ay_filt_unbiased'] = data['ay_filt'] - data['ekf_acc_b_y']
            data['az_filt_unbiased'] = data['az_filt'] - data['ekf_acc_b_z']
    
        # VIO
        if 'vioPos[0]' in data and False:
            print('vioooo')
            print(np.any(data['vioPos[0]'] != 0))
            data['x_vio'] = data.pop('vioPos[0]')*1e-3 # mm to m
            data['y_vio'] = data.pop('vioPos[1]')*1e-3
            data['z_vio'] = data.pop('vioPos[2]')*1e-3
            data['vx_vio'] = data.pop('vioVel[0]')*1e-2 # cm/s to m/s
            data['vy_vio'] = data.pop('vioVel[1]')*1e-2
            data['vz_vio'] = data.pop('vioVel[2]')*1e-2
            data['qw_vio'] = data.pop('vioQuat[0]')/quat_scaling
            data['qx_vio'] = data.pop('vioQuat[1]')/quat_scaling
            data['qy_vio'] = data.pop('vioQuat[2]')/quat_scaling
            data['qz_vio'] = data.pop('vioQuat[3]')/quat_scaling
            data['phi_vio'] = np.arctan2(2*(data['qw_vio']*data['qx_vio'] + data['qy_vio']*data['qz_vio']), 1 - 2*(data['qx_vio']**2 + data['qy_vio']**2))
            data['theta_vio'] = np.arcsin(2*(data['qw_vio']*data['qy_vio'] - data['qz_vio']*data['qx_vio']))
            data['psi_vio'] = np.arctan2(2*(data['qw_vio']*data['qz_vio'] + data['qx_vio']*data['qy_vio']), 1 - 2*(data['qy_vio']**2 + data['qz_vio']**2))
            data['p_vio'] = data.pop('vioRate[0]')*np.pi/180 # deg/s to rad/s
            data['q_vio'] = data.pop('vioRate[1]')*np.pi/180
            data['r_vio'] = data.pop('vioRate[2]')*np.pi/180

        # UGLY HACK: overwrite states with ekf states
        # if 'ekf_x' in data.keys():
        #     print('WARNING: overwriting states with ekf states')
        #     data['x'] = data['ekf_x']
        #     data['y'] = data['ekf_y']
        #     data['z'] = data['ekf_z']
        #     data['vx'] = data['ekf_vx']
        #     data['vy'] = data['ekf_vy']
        #     data['vz'] = data['ekf_vz']
        #     data['phi'] = data['ekf_phi']
        #     data['theta'] = data['ekf_theta']
        #     data['psi'] = data['ekf_psi']
        
        # EXTRA
        data['v'] = np.sqrt(data['vx']**2 + data['vy']**2 + data['vz']**2)

        # body velocities
        v_body = np.stack([
            Rmat(phi, theta, psi).T@[vx, vy, vz]
            for vx, vy, vz, phi, theta, psi
            in zip(data['vx'],data['vy'],data['vz'],data['phi'],data['theta'],data['psi'])
        ])
        data['vbx'] = v_body[:,0]
        data['vby'] = v_body[:,1]
        data['vbz'] = v_body[:,2]
        
        # simple drag model
        data['Dx'] = -0.43291866*data['vbx']
        data['Dy'] = -0.49557959*data['vby']
        
        # world accelerations
        a_world = np.stack([
            Rmat(phi, theta, psi)@np.array([ax, ay, az]) + np.array([0, 0, 9.81])
            for ax, ay, az, phi, theta, psi
            in zip(data['ax'],data['ay'],data['az'],data['phi'],data['theta'],data['psi'])
        ])
        data['awx'] = a_world[:,0]
        data['awy'] = a_world[:,1]
        data['awz'] = a_world[:,2]
        
        data['awx_filt'] = sp.signal.sosfiltfilt(sos, data['awx'])
        data['awy_filt'] = sp.signal.sosfiltfilt(sos, data['awy'])
        data['awz_filt'] = sp.signal.sosfiltfilt(sos, data['awz'])
        
        # # reconstruct acc setpoint by rotating the thrust setpoint with the attitude setpoint
        # a_world_sp_rec = np.stack([
        #     Rmat(phi, theta, psi)@np.array([0,0,T]) + np.array([0, 0, 9.81])
        #     for T, phi, theta, psi
        #     in zip(data['T_sp'],data['phi_sp'],data['theta_sp'],data['psi_sp'])
        # ])
        # data['awx_sp_rec'] = a_world_sp_rec[:, 0]
        # data['awy_sp_rec'] = a_world_sp_rec[:, 1]
        # data['awz_sp_rec'] = a_world_sp_rec[:, 2]
        
        # # test att, thrust setpoint calculation
        # att_sp_test = np.array([
        #     att_thrust_sp_from_acc(np.array([qw,qx,qy,qz]), np.array([awx,awy,awz]))[0] for 
        #     qw,qx,qy,qz,awx,awy,awz in zip(data['qw'],data['qx'],data['qy'],data['qz'],data['awx_sp'],data['awy_sp'],data['awz_sp'])
        # ])
        
        # data['qw_sp_test'] = att_sp_test[:, 0]
        # data['qx_sp_test'] = att_sp_test[:, 1]
        # data['qy_sp_test'] = att_sp_test[:, 2]
        # data['qz_sp_test'] = att_sp_test[:, 3]
        
        # data['phi_sp_test'] = np.arctan2(2*(data['qw_sp_test']*data['qx_sp_test'] + data['qy_sp_test']*data['qz_sp_test']), 1 - 2*(data['qx_sp_test']**2 + data['qy_sp_test']**2))
        # data['theta_sp_test'] = np.arcsin(2*(data['qw_sp_test']*data['qy_sp_test'] - data['qz_sp_test']*data['qx_sp_test']))
        # data['psi_sp_test'] = np.arctan2(2*(data['qw_sp_test']*data['qz_sp_test'] + data['qx_sp_test']*data['qy_sp_test']), 1 - 2*(data['qy_sp_test']**2 + data['qz_sp_test']**2))
        
        # # Thrust setpoint
        # thrust_sp_test = np.array([
        #     att_thrust_sp_from_acc(np.array([qw,qx,qy,qz]), np.array([awx,awy,awz]))[1] for 
        #     qw,qx,qy,qz,awx,awy,awz in zip(data['qw'],data['qx'],data['qy'],data['qz'],data['awx_sp'],data['awy_sp'],data['awz_sp'])
        # ])
        # data['T_sp_test'] = thrust_sp_test
        
        # # Rate setpoint
        # rate_sp_test = np.array([
        #     att_thrust_sp_from_acc(np.array([qw,qx,qy,qz]), np.array([awx,awy,awz]))[2] for 
        #     qw,qx,qy,qz,awx,awy,awz in zip(data['qw'],data['qx'],data['qy'],data['qz'],data['awx_sp'],data['awy_sp'],data['awz_sp'])
        # ])
        # data['p_sp_test'] = rate_sp_test[:, 0]
        # data['q_sp_test'] = rate_sp_test[:, 1]
        # data['r_sp_test'] = rate_sp_test[:, 2]
        
        # # reconstruct acc setpoint from test att, thrust setpoint
        # a_world_sp_rec_test = np.stack([
        #     Rmat(phi, theta, psi)@np.array([0,0,T]) + np.array([0, 0, 9.81])
        #     for T, phi, theta, psi
        #     in zip(data['T_sp_test'],data['phi_sp_test'],data['theta_sp_test'],data['psi_sp_test'])
        # ])
        # data['awx_sp_rec_test'] = a_world_sp_rec_test[:, 0]
        # data['awy_sp_rec_test'] = a_world_sp_rec_test[:, 1]
        # data['awz_sp_rec_test'] = a_world_sp_rec_test[:, 2]
        
        return data

# DATA TRIM FUNCTIONS
def trim_nn_active(data):
    if 'nn_active' in data.keys():
        indices = data['nn_active']>0.
    else:
        indices = data['flightModeFlags'] > 8000
    data = {k: v[indices] for k, v in data.items() if isinstance(v, np.ndarray) and len(v) == len(indices)}
    data['t'] = data['t'] - data['t'][0]
    return data

def trim_time(data, t0=0, tf=12):
    indices = (data['t'] >= t0) & (data['t'] <= tf)
    try:
        # add one more data point to make sure that t=tf is included
        i = np.min(np.where(data['t']>tf)[0])
        indices[i] = True
    except:
        print("Warning: tf is greater than the last time point")
    data = {k: v[indices] for k, v in data.items() if isinstance(v, np.ndarray) and len(v) == len(indices)}
    data['t'] = data['t'] - data['t'][0]
    return data

def split_where_nn_active(data):
    # plt.plot(data['flightModeFlags'])
    # plt.show()
    nn_active = data['flightModeFlags'] > 8000
    nn_activate = [i for i in range(1, len(nn_active)) if nn_active[i] and not nn_active[i-1]]
    nn_deactivate = [i for i in range(1, len(nn_active)) if not nn_active[i] and nn_active[i-1]]
    # make sure nn_activate and nn_deactivate are the same length
    if len(nn_activate) > len(nn_deactivate):
        nn_deactivate.append(len(nn_active))
    split_data = []
    for i in range(len(nn_activate)):
        split_data.append({k: v[nn_activate[i]:nn_deactivate[i]] for k, v in data.items() if isinstance(v, np.ndarray) and len(v) == len(data['t'])})
        split_data[-1]['t'] = split_data[-1]['t'] - split_data[-1]['t'][0]
    return split_data


# FUNCTIONS FOR ANIMATIONS  
from quadcopter_animation import animation
import importlib
importlib.reload(animation)

# RACE TRACK:
r=3.
num = 8
gate_pos = np.array([
    [r*np.cos(angle), r*np.sin(angle), -1.5] for angle in np.linspace(0,2*np.pi,num,endpoint=False)
])
gate_yaw = np.array([np.pi/2 + i*2*np.pi/num for i in range(num)])

def animate_data(data):
    animation.animate(
        data['t'],
        data['x'], data['y'], data['z'],
        data['phi'], data['theta'], data['psi'],
        np.stack([data['u1'], data['u2'], data['u3'], data['u4']], axis=1),
        target=np.stack([data['x_sp'], data['y_sp'], data['z_sp']], axis=1)
    )
    
def animate_data_double(data1, data2):
    animation.animate(
        [data1['t'], data2['t']],
        [data1['x'], data2['x']],
        [data1['y'], data2['y']],
        [data1['z'], data2['z']],
        [data1['phi'], data2['phi']],
        [data1['theta'], data2['theta']],
        [data1['psi'], data2['psi']],
        [
            np.stack([data1['u1'], data1['u2'], data1['u3'], data1['u4']], axis=1),
            np.stack([data2['u1'], data2['u2'], data2['u3'], data2['u4']], axis=1)
        ],
        multiple_trajectories=True,
        simultaneous=True,
        colors=[(255,0,0), (0,255,0)]
    )
    
def animate_data_multiple(*data_list, **kwargs):
    if 'colors' in kwargs:
        colors_ = kwargs.pop('colors')
        colors = lambda i: colors_[i]
    else:
        # color map
        import matplotlib.cm as cm
        colors_ = cm.get_cmap('jet', len(data_list))
        # to int
        colors = lambda i: tuple(int(c*255) for c in colors_(i)[:-1])
    
    animation.animate(
        [d['t'] for d in data_list],
        [d['x'] for d in data_list],
        [d['y'] for d in data_list],
        [d['z'] for d in data_list],
        [d['phi'] for d in data_list],
        [d['theta'] for d in data_list],
        [d['psi'] for d in data_list],
        [np.stack([d['u1'], d['u2'], d['u3'], d['u4']], axis=1) for d in data_list],
        # target=[np.stack([d['x_opti'], d['y_opti'], d['z_opti']], axis=1) for d in data_list],
        multiple_trajectories=True,
        simultaneous=True,
        colors=[colors(i) for i in range(len(data_list))],
        **kwargs
    )
    
def animate_data_multiple2(*data_list, **kwargs):
    # color map
    import matplotlib.cm as cm
    colors_ = cm.get_cmap('jet', len(data_list))
    # to int
    colors = lambda i: tuple(int(c*255) for c in colors_(i)[:-1])
    colors2 = lambda i: tuple(int(c*55+200) for c in colors_(i)[:-1])
    
    animation.animate(
        [d['t'] for d in data_list]*2,
        [d['x'] for d in data_list] + [d['x_opti'] for d in data_list],
        [d['y'] for d in data_list] + [d['y_opti'] for d in data_list],
        [d['z'] for d in data_list] + [d['z_opti'] for d in data_list],
        [d['phi'] for d in data_list] + [d['phi_opti'] for d in data_list],
        [d['theta'] for d in data_list] + [d['theta_opti'] for d in data_list],
        [d['psi'] for d in data_list] + [d['psi_opti'] for d in data_list],
        [np.stack([d['u1'], d['u2'], d['u3'], d['u4']], axis=1) for d in data_list]+[np.zeros((len(data_list[0]['t']), 4)) for _ in data_list],
        target=np.stack([data_list[0]['x_sp'], data_list[0]['y_sp'], data_list[0]['z_sp']], axis=1),
        multiple_trajectories=True,
        simultaneous=True,
        colors=[colors(i) for i in range(len(data_list))]+[colors2(i) for i in range(len(data_list))],
        **kwargs,
    )
    
    
def animate_data2(data):
    animation.animate(
        [data['t'], data['t']],
        [data['x'], data['x_opti']],
        [data['y'], data['y_opti']],
        [data['z'], data['z_opti']],
        [data['phi'], data['phi_opti']],
        [data['theta'], data['theta_opti']],
        [data['psi'], data['psi_opti']],
        [
            np.stack([data['u1'], data['u2'], data['u3'], data['u4']], axis=1),
            np.stack([data['u1'], data['u2'], data['u3'], data['u4']], axis=1)
        ],
        multiple_trajectories=True,
        simultaneous=True,
        target=np.stack([data['x_sp'], data['y_sp'], data['z_sp']], axis=1),
        colors=[(255,0,0), (0,255,0)]
    )
    
# FUNCTIONS FOR SYSTEM IDENTIFICATION
def fit_thrust_drag_model(data, subtract_ekf_bias=True, new_model=False):
    print('fitting thrust and drag model')
    fig, axs = plt.subplots(1, 3, figsize=(10, 10), sharex=True, sharey=True)
    
    # THRUST MODEL ------------------------------------------------------------------------------
    # az = k_w*sum(omega_i**2)
    # we will find k_w, by linear regression
    X = np.stack([
        data['omega[0]']**2 + data['omega[1]']**2 + data['omega[2]']**2 + data['omega[3]']**2,
        # data['vbx']**2 + data['vby']**2,
        # data['vz']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])
    ])
    Y = data['az_unfiltered']
    if subtract_ekf_bias:
        Y = data['az'] - data['ekf_acc_b_z']
    k_w, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    
    # 2nd thrust model for ref
    X2 = np.stack([
        data['omega[0]']**2 + data['omega[1]']**2 + data['omega[2]']**2 + data['omega[3]']**2,
        data['vbx']**2 + data['vby']**2,
        data['vz']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])
    ])
    k_w2, k_h2, k_z2 = A2 = np.linalg.lstsq(X2.T, Y, rcond=None)[0]
    
    radius = 0.127*0.8 / 2
    average_omega = (data['omega[0]'] + data['omega[1]'] + data['omega[2]'] + data['omega[3]']) / 4
    angle_of_attack = np.arctan2(data['vbz'], radius*average_omega)
    omega_omega = data['omega[0]']**2 + data['omega[1]']**2 + data['omega[2]']**2 + data['omega[3]']**2
    v2 = data['vbz']*np.abs(data['vbz'])
    # mu_x = np.(data['vbx'] / (radius*average_omega))
    # mu_y = np.abs(data['vby'] / (radius*average_omega))
    mu_xxyy = np.arctan2(data['vbx']**2+data['vby']**2, (radius*average_omega)**2)
    X3 = np.stack([
        omega_omega,
        angle_of_attack*omega_omega
    ])
    Y3 = data['az']
    A3 = np.linalg.lstsq(X3.T, Y3, rcond=None)[0]
    # print(A3)
    A3 = np.array([-1.94215099e-06, -6.74241812e-06])
    
    # [-1.94215099e-06 -6.74241812e-06]
    # [-1.94904262e-06 -5.46349654e-06  1.44906622e-02]
    # [-2.06168526e-06 -5.56942980e-06  1.63245116e-02  1.31299062e-13]
    X4 = np.stack([
        omega_omega,
        angle_of_attack*omega_omega,
        v2
    ])
    A4 = np.linalg.lstsq(X4.T, Y3, rcond=None)[0]
    # print(A4)
    A4 = np.array([-1.94904262e-06, -5.46349654e-06,  1.44906622e-02])
    
    X5 = np.stack([
        omega_omega,
        angle_of_attack*omega_omega,
        v2,
        mu_xxyy*omega_omega
    ])
    
    A5 = np.linalg.lstsq(X5.T, Y3, rcond=None)[0]
    
    k_w = -1.55e-6
    k_angle = 3.14514
    k_hor = 7.245
    k_z = 0.00
    erin_fit = k_w*(1+angle_of_attack*k_angle + k_hor*mu_xxyy)*omega_omega + k_z*v2
    
    def new_thrust_model(k_w, k_angle, k_hor, k_z):
        return k_w*(1+angle_of_attack*k_angle + k_hor*mu_xxyy)*omega_omega + k_z*v2
    
    # 1000 fits with 30% randomized parameters
    r =0.2
    az_fits = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        # generate random parameters
        Anom = np.array([k_w, k_angle, k_hor, k_z])
        Arandom = Anom * np.random.uniform(1-r, 1+r, 4)
        az_fits[i] = new_thrust_model(*Arandom)
    az_fit_high = np.max(az_fits, axis=0)
    az_fit_low = np.min(az_fits, axis=0)
    
    
    if 'az_unfiltered' in data:
        axs[0].plot(data['t'], data['az_unfiltered'], label='az raw', alpha=0.1, color='blue')
    axs[0].plot(data['t'], Y, label='az') #, alpha=0.2)
    # axs[0].plot(data['t'], data['az_filt'], label='az filt')

    axs[0].plot(data['t'], A@X, label='T model')
    Ahigh = A*1.3
    Alow = A*0.7
    # axs[0].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # print MAE
    print('MAE')
    print('model fit:', np.mean(np.abs(Y - A@X)))

    axs[0].plot(data['t'], erin_fit, label='T model erin')
    axs[0].fill_between(data['t'], az_fit_low, az_fit_high, alpha=0.2, color='green')
    # MAE
    print('model erin:', np.mean(np.abs(Y - erin_fit)))
        

    # axs[0].plot(data['t'], A_nom@X, label='T model nominal')
    axs[0].set_xlabel('t [s]')
    axs[0].set_ylabel('acc [m/s^2]')
    axs[0].legend()
    axs[0].set_title('Thrust model: \n az = k_w*sum(omega_i**2) \n k_w = {:.2e}'.format(A[0]))
    # axs[0].set_title('Thrust model: \n az = k_w*sum(omega_i**2) + k_h*(vbx**2+vby**2) + k_z*vbz*sum(omega_i) \n k_w, k_h, k_z = {:.2e}, {:.2e}, {:.2e}'.format(k_w, k_h, k_z))
    
    # DRAG MODEL X ------------------------------------------------------------------------------
    # Eq. 2 from https://doi.org/10.1016/j.robot.2023.104588
    # ax = -k_x*vbx*sum(omega_i)
    # we will find k_x by linear regression
    X = np.stack([data['vbx']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])])
    # X = np.stack([data['vbx']])
    Y = data['ax']
    if subtract_ekf_bias:
        Y = data['ax'] - data['ekf_acc_b_x']
    k_x, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    
    if 'ax_unfiltered' in data:
        axs[1].plot(data['t'], data['ax_unfiltered'], label='ax raw', alpha=0.1, color='blue')
    axs[1].plot(data['t'], Y, label='ax') #, alpha=0.2)
    # axs[1].plot(data['t'], data['ax_filt'], label='ax filt')
    axs[1].plot(data['t'], A@X, label='Dx model')
    Ahigh = A*1.3
    Alow = A*0.7
    axs[1].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # axs[1].plot(data['t'], A_nom@X, label='Dx model nominal')
    axs[1].set_xlabel('t [s]')
    axs[1].set_ylabel('acc [m/s^2]')
    axs[1].legend()
    axs[1].set_title('Drag model X: \n ax = k_x*vbx*sum(omega_i) \n k_x = {:.2e}'.format(k_x))
    
    # DRAG MODEL Y ------------------------------------------------------------------------------
    # Eq. 2 from https://doi.org/10.1016/j.robot.2023.104588
    # ay = -k_y*vby*sum(omega_i)
    # we will find k_y by linear regression
    X = np.stack([data['vby']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])])
    # X = np.stack([data['vby']])
    Y = data['ay']
    if subtract_ekf_bias:
        Y = data['ay'] - data['ekf_acc_b_y']
    k_y, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    
    if 'ay_unfiltered' in data:
        axs[2].plot(data['t'], data['ay_unfiltered'], label='ay raw', alpha=0.1, color='blue')
    axs[2].plot(data['t'], Y, label='ay') #, alpha=0.2)
    # axs[2].plot(data['t'], data['ay_filt'], label='ay filt')
    axs[2].plot(data['t'], A@X, label='Dy model')
    Ahigh = A*1.3
    Alow = A*0.7
    axs[2].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # axs[2].plot(data['t'], A_nom@X, label='Dy model nominal')
    axs[2].set_xlabel('t [s]')
    axs[2].set_ylabel('acc [m/s^2]')
    axs[2].legend()
    axs[2].set_title('Drag model Y: \n ay = k_y*vby*sum(omega_i) \n k_y = {:.2e}'.format(k_y))
    
    # show fig with the window name 'Thrust and Drag Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Thrust and Drag Model')
    plt.show()
    
    # print('k_w = {:.2e}, k_x = {:.2e}, k_y = {:.2e}'.format(k_w, k_x, k_y))
    return k_w, k_x, k_y

def plot_thrust_drag_model(data, params, subtract_ekf_bias=True, randomization=30):
    print('fitting thrust and drag model')
    fig, axs = plt.subplots(1, 3, figsize=(10, 10), sharex=True, sharey=True)
    
    # THRUST MODEL ------------------------------------------------------------------------------
    # az = k_w*sum(omega_i**2)
    # we will find k_w, by linear regression
    X = np.stack([
        data['omega[0]']**2 + data['omega[1]']**2 + data['omega[2]']**2 + data['omega[3]']**2,
        # data['vbx']**2 + data['vby']**2,
        # data['vz']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])
    ])
    Y = data['az_unfiltered']
    if subtract_ekf_bias:
        Y = data['az'] - data['ekf_acc_b_z']
    # k_w, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    A = np.array([params['k_w']])
    k_w, = A
    
    # 2nd thrust model for ref
    X2 = np.stack([
        data['omega[0]']**2 + data['omega[1]']**2 + data['omega[2]']**2 + data['omega[3]']**2,
        data['vbx']**2 + data['vby']**2,
        data['vz']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])
    ])
    k_w2, k_h2, k_z2 = A2 = np.linalg.lstsq(X2.T, Y, rcond=None)[0]
    
    if 'az_unfiltered' in data:
        axs[0].plot(data['t'], data['az_unfiltered'], label='az raw', alpha=0.1, color='blue')
    axs[0].plot(data['t'], Y, label='az') #, alpha=0.2)
    # axs[0].plot(data['t'], data['az_filt'], label='az filt')
    axs[0].plot(data['t'], A@X, label='T model')
    r = randomization/100
    Ahigh = A*(1+r)
    Alow = A*(1-r)
    axs[0].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # axs[0].plot(data['t'], A2@X2, label='T model 2')
    # axs[0].plot(data['t'], A_nom@X, label='T model nominal')
    axs[0].set_xlabel('t [s]')
    axs[0].set_ylabel('acc [m/s^2]')
    axs[0].legend()
    axs[0].set_title('Thrust model: \n az = k_w*sum(omega_i**2) \n k_w = {:.2e}'.format(k_w))
    # axs[0].set_title('Thrust model: \n az = k_w*sum(omega_i**2) + k_h*(vbx**2+vby**2) + k_z*vbz*sum(omega_i) \n k_w, k_h, k_z = {:.2e}, {:.2e}, {:.2e}'.format(k_w, k_h, k_z))
    
    # DRAG MODEL X ------------------------------------------------------------------------------
    # Eq. 2 from https://doi.org/10.1016/j.robot.2023.104588
    # ax = -k_x*vbx*sum(omega_i)
    # we will find k_x by linear regression
    X = np.stack([data['vbx']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])])
    # X = np.stack([data['vbx']])
    Y = data['ax']
    if subtract_ekf_bias:
        Y = data['ax'] - data['ekf_acc_b_x']
    # k_x, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    A = np.array([params['k_x']])
    k_x, = A
    
    if 'ax_unfiltered' in data:
        axs[1].plot(data['t'], data['ax_unfiltered'], label='ax raw', alpha=0.1, color='blue')
    axs[1].plot(data['t'], Y, label='ax') #, alpha=0.2)
    # axs[1].plot(data['t'], data['ax_filt'], label='ax filt')
    axs[1].plot(data['t'], A@X, label='Dx model')
    Ahigh = A*(1+r)
    Alow = A*(1-r)
    axs[1].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # axs[1].plot(data['t'], A_nom@X, label='Dx model nominal')
    axs[1].set_xlabel('t [s]')
    axs[1].set_ylabel('acc [m/s^2]')
    axs[1].legend()
    axs[1].set_title('Drag model X: \n ax = k_x*vbx*sum(omega_i) \n k_x = {:.2e}'.format(k_x))
    
    # DRAG MODEL Y ------------------------------------------------------------------------------
    # Eq. 2 from https://doi.org/10.1016/j.robot.2023.104588
    # ay = -k_y*vby*sum(omega_i)
    # we will find k_y by linear regression
    X = np.stack([data['vby']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]'])])
    # X = np.stack([data['vby']])
    Y = data['ay']
    if subtract_ekf_bias:
        Y = data['ay'] - data['ekf_acc_b_y']
    # k_y, = A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    A = np.array([params['k_y']])
    k_y, = A
    
    if 'ay_unfiltered' in data:
        axs[2].plot(data['t'], data['ay_unfiltered'], label='ay raw', alpha=0.1, color='blue')
    axs[2].plot(data['t'], Y, label='ay') #, alpha=0.2)
    # axs[2].plot(data['t'], data['ay_filt'], label='ay filt')
    axs[2].plot(data['t'], A@X, label='Dy model')
    Ahigh = A*(1+r)
    Alow = A*(1-r)
    axs[2].fill_between(data['t'], Alow@X, Ahigh@X, alpha=0.2, color='orange')
    # axs[2].plot(data['t'], A_nom@X, label='Dy model nominal')
    axs[2].set_xlabel('t [s]')
    axs[2].set_ylabel('acc [m/s^2]')
    axs[2].legend()
    axs[2].set_title('Drag model Y: \n ay = k_y*vby*sum(omega_i) \n k_y = {:.2e}'.format(k_y))
    
    # show fig with the window name 'Thrust and Drag Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Thrust and Drag Model')
    plt.show()
    
    # print('k_w = {:.2e}, k_x = {:.2e}, k_y = {:.2e}'.format(k_w, k_x, k_y))
    return k_w, k_x, k_y

from scipy.optimize import minimize

def fit_actuator_model(data, raw_motor_speed=False):
    # the steadystate rpm motor response to the motor command u is described by:
    # w_c = (w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min
    # the dynamics of the motor is described by:
    # dw/dt = (w_c - w)/tau
    # dw/dt = ((w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min - w)*tau_inv
    # we will find w_min, w_max, k, tau_inv by nonlinear optimization
    
    def get_w_est(params, u, w):
        w_min, w_max, k, tau_inv = params
        w_c = (w_max-w_min)*np.sqrt(k*u**2 + (1-k)*u) + w_min
        # progate the dynamics
        w_est = np.zeros_like(u)
        w_est[0] = w[0]
        for i in range(1, len(w_est)):
            dt = data['t'][i] - data['t'][i-1]
            w_est[i] = w_est[i-1] + (w_c[i] - w_est[i-1])*dt*tau_inv
        return w_est

    def get_error(params, u, w):
        return np.linalg.norm(get_w_est(params, u, w) - w)
    
    # w_min, w_max, k, tau_inv
    initial_guess = [285, 2700, 0.75, 100]
    bounds = [(0, 1000), (0, 6000), (0, 1), (1, 1000.)]
    
    # minimize for each motor
    if raw_motor_speed:
        err_1 = lambda x: get_error(x, data['u1'], data['omegaUnfiltered[0]'])
        err_2 = lambda x: get_error(x, data['u2'], data['omegaUnfiltered[1]'])
        err_3 = lambda x: get_error(x, data['u3'], data['omegaUnfiltered[2]'])
        err_4 = lambda x: get_error(x, data['u4'], data['omegaUnfiltered[3]'])
    else:
        err_1 = lambda x: get_error(x, data['u1'], data['omega[0]'])
        err_2 = lambda x: get_error(x, data['u2'], data['omega[1]'])
        err_3 = lambda x: get_error(x, data['u3'], data['omega[2]'])
        err_4 = lambda x: get_error(x, data['u4'], data['omega[3]'])
    err_tot = lambda x: err_1(x) + err_2(x) + err_3(x) + err_4(x)
    
    print('fitting actuator model...')
    res_1 = minimize(err_1, initial_guess, bounds=bounds)
    res_2 = minimize(err_2, initial_guess, bounds=bounds)
    res_3 = minimize(err_3, initial_guess, bounds=bounds)
    res_4 = minimize(err_4, initial_guess, bounds=bounds)
    res_tot = minimize(err_tot, initial_guess, bounds=bounds)
    
    # set k to 45
    # res_tot.x[2] = 0.45
    
    # plot results
    fig, axs = plt.subplots(2, 2, figsize=(10, 10), sharex=True, sharey=True)
    
    axs[0,0].plot(data['t'], data['omega[0]'], label='w')
    axs[0,0].plot(data['t'], get_w_est(res_1.x, data['u1'], data['omega[0]']), label='w est')
    axs[0,0].plot(data['t'], get_w_est(res_tot.x, data['u1'], data['omega[0]']), label='w est tot')
    # axs[0,0].plot(data['t'], get_w_est(res_nom, data['u1'], data['omega[0]']), label='w est nom')
    axs[0,0].set_xlabel('t [s]')
    axs[0,0].set_ylabel('w [rad/s]')
    axs[0,0].legend()
    params = res_1.x
    params[3] = 1/params[3]
    axs[0,0].set_title('Motor 1: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[0,1].plot(data['t'], data['omega[1]'], label='w')
    axs[0,1].plot(data['t'], get_w_est(res_2.x, data['u2'], data['omega[1]']), label='w_est')
    axs[0,1].plot(data['t'], get_w_est(res_tot.x, data['u2'], data['omega[1]']), label='w est tot')
    # axs[0,1].plot(data['t'], get_w_est(res_nom, data['u2'], data['omega[1]']), label='w est nom')
    axs[0,1].set_xlabel('t [s]')
    axs[0,1].set_ylabel('w [rad/s]')
    axs[0,1].legend()
    params = res_2.x
    params[3] = 1/params[3]
    axs[0,1].set_title('Motor 2: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[1,0].plot(data['t'], data['omega[2]'], label='w')
    axs[1,0].plot(data['t'], get_w_est(res_3.x, data['u3'], data['omega[2]']), label='w_est')
    axs[1,0].plot(data['t'], get_w_est(res_tot.x, data['u3'], data['omega[2]']), label='w est tot')
    # axs[1,0].plot(data['t'], get_w_est(res_nom, data['u3'], data['omega[2]']), label='w est nom')
    axs[1,0].set_xlabel('t [s]')
    axs[1,0].set_ylabel('w [rad/s]')
    axs[1,0].legend()
    params = res_3.x
    params[3] = 1/params[3]
    axs[1,0].set_title('Motor 3: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[1,1].plot(data['t'], data['omega[3]'], label='w')
    axs[1,1].plot(data['t'], get_w_est(res_4.x, data['u4'], data['omega[3]']), label='w_est')
    axs[1,1].plot(data['t'], get_w_est(res_tot.x, data['u4'], data['omega[3]']), label='w est tot')
    # axs[1,1].plot(data['t'], get_w_est(res_nom, data['u4'], data['omega[3]']), label='w est nom')
    axs[1,1].set_xlabel('t [s]')
    axs[1,1].set_ylabel('w [rad/s]')
    axs[1,1].legend()
    params = res_4.x
    params[3] = 1/params[3]
    axs[1,1].set_title('Motor 4: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    # suptitle
    params = res_tot.x
    params[3] = 1/params[3]
    fig.suptitle('Actuator model: \n dw/dt = dw/dt = ((w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min - w)/tau \n Total fit: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    # show fig with the window name 'Actuator Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Actuator Model')
    plt.show()
    
    # print('w_min={:.2f}, w_max={:.2f}, k={:.2f}, tau={:.2f}'.format(*res_tot.x))
    return res_tot.x

def plot_actuator_model(data, params, raw_motor_speed=False, randomization=30):
    # the steadystate rpm motor response to the motor command u is described by:
    # w_c = (w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min
    # the dynamics of the motor is described by:
    # dw/dt = (w_c - w)/tau
    # dw/dt = ((w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min - w)*tau_inv
    # we will find w_min, w_max, k, tau_inv by nonlinear optimization
    
    def get_w_est(params, u, w):
        w_min, w_max, k, tau_inv = params
        w_c = (w_max-w_min)*np.sqrt(k*u**2 + (1-k)*u) + w_min
        # progate the dynamics
        w_est = np.zeros_like(u)
        w_est[0] = w[0]
        for i in range(1, len(w_est)):
            dt = data['t'][i] - data['t'][i-1]
            w_est[i] = w_est[i-1] + (w_c[i] - w_est[i-1])*dt*tau_inv
        return w_est

    def get_error(params, u, w):
        return np.linalg.norm(get_w_est(params, u, w) - w)
    
    # plot results
    fig, axs = plt.subplots(2, 2, figsize=(10, 10), sharex=True, sharey=True)
    
    res = np.array([params['w_min'], params['w_max'], params['k'], 1/params['tau']])
    
    axs[0,0].plot(data['t'], data['omega[0]'], label='w')
    axs[0,0].plot(data['t'], get_w_est(res, data['u1'], data['omega[0]']), label='w est')
    # 1000 fits with r% parameter variation
    r = randomization/100
    w_fits = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        w_min = res[0] * np.random.uniform(1-r, 1+r)
        w_max = res[1] * np.random.uniform(1-r, 1+r)
        k = np.random.uniform(res[2]*(1-r), min(res[2]*(1+r), 1))
        tau = (1/res[3])*np.random.uniform(1-r, 1+r)
        w_fits[i] = get_w_est([w_min, w_max, k, 1/tau], data['u1'], data['omega[0]'])
    w_fit_high = np.max(w_fits, axis=0)
    w_fit_low = np.min(w_fits, axis=0)
    axs[0,0].fill_between(data['t'], w_fit_low, w_fit_high, alpha=0.2, color='orange')
        
        
    axs[0,0].set_xlabel('t [s]')
    axs[0,0].set_ylabel('w [rad/s]')
    axs[0,0].legend()
    params = res.copy()
    params[3] = 1/params[3]
    axs[0,0].set_title('Motor 1: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[0,1].plot(data['t'], data['omega[1]'], label='w')
    axs[0,1].plot(data['t'], get_w_est(res, data['u2'], data['omega[1]']), label='w_est')
    # 1000 fits with r% parameter variation
    r = randomization/100
    w_fits = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        w_min = res[0] * np.random.uniform(1-r, 1+r)
        w_max = res[1] * np.random.uniform(1-r, 1+r)
        k = np.random.uniform(res[2]*(1-r), min(res[2]*(1+r), 1))
        tau = (1/res[3])*np.random.uniform(1-r, 1+r)
        w_fits[i] = get_w_est([w_min, w_max, k, 1/tau], data['u2'], data['omega[1]'])
    w_fit_high = np.max(w_fits, axis=0)
    w_fit_low = np.min(w_fits, axis=0)
    axs[0,1].fill_between(data['t'], w_fit_low, w_fit_high, alpha=0.2, color='orange')
    
    axs[0,1].set_xlabel('t [s]')
    axs[0,1].set_ylabel('w [rad/s]')
    axs[0,1].legend()
    params = res.copy()
    params[3] = 1/params[3]
    axs[0,1].set_title('Motor 2: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[1,0].plot(data['t'], data['omega[2]'], label='w')
    axs[1,0].plot(data['t'], get_w_est(res, data['u3'], data['omega[2]']), label='w_est')
    # 1000 fits with r% parameter variation
    r = randomization/100
    w_fits = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        w_min = res[0] * np.random.uniform(1-r, 1+r)
        w_max = res[1] * np.random.uniform(1-r, 1+r)
        k = np.random.uniform(res[2]*(1-r), min(res[2]*(1+r), 1))
        tau = (1/res[3])*np.random.uniform(1-r, 1+r)
        w_fits[i] = get_w_est([w_min, w_max, k, 1/tau], data['u3'], data['omega[2]'])
    w_fit_high = np.max(w_fits, axis=0)
    w_fit_low = np.min(w_fits, axis=0)
    axs[1,0].fill_between(data['t'], w_fit_low, w_fit_high, alpha=0.2, color='orange')
        
    axs[1,0].set_xlabel('t [s]')
    axs[1,0].set_ylabel('w [rad/s]')
    axs[1,0].legend()
    params = res.copy()
    params[3] = 1/params[3]
    axs[1,0].set_title('Motor 3: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    axs[1,1].plot(data['t'], data['omega[3]'], label='w')
    axs[1,1].plot(data['t'], get_w_est(res, data['u4'], data['omega[3]']), label='w_est')
    # 1000 fits with r% parameter variation
    r = randomization/100
    w_fits = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        w_min = res[0] * np.random.uniform(1-r, 1+r)
        w_max = res[1] * np.random.uniform(1-r, 1+r)
        k = np.random.uniform(res[2]*(1-r), min(res[2]*(1+r), 1))
        tau = (1/res[3])*np.random.uniform(1-r, 1+r)
        w_fits[i] = get_w_est([w_min, w_max, k, 1/tau], data['u4'], data['omega[3]'])
    w_fit_high = np.max(w_fits, axis=0)
    w_fit_low = np.min(w_fits, axis=0)
    axs[1,1].fill_between(data['t'], w_fit_low, w_fit_high, alpha=0.2, color='orange')
    
    axs[1,1].set_xlabel('t [s]')
    axs[1,1].set_ylabel('w [rad/s]')
    axs[1,1].legend()
    params = res.copy()
    params[3] = 1/params[3]
    axs[1,1].set_title('Motor 4: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    # suptitle
    params = res.copy()
    params[3] = 1/params[3]
    fig.suptitle('Actuator model: \n dw/dt = dw/dt = ((w_max-w_min)*sqrt(k u**2 + (1-k)*u) + w_min - w)/tau \n Total fit: w_min = {:.2f}, w_max = {:.2f}, k = {:.2f}, tau = {:.2f}'.format(*params))
    
    # show fig with the window name 'Actuator Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Actuator Model')
    plt.show()
    
    # print('w_min={:.2f}, w_max={:.2f}, k={:.2f}, tau={:.2f}'.format(*res_tot.x))
    return res

def fit_moments_model(data):
    print('fitting moments model')
    # model from https://doi.org/10.1016/j.robot.2023.104588
    # d_p     = (q*r*(Iyy-Izz) + Mx)/Ixx = Jx*q*r + Mx_
    # d_q     = (p*r*(Izz-Ixx) + My)/Iyy = Jy*p*r + My_
    # d_r     = (p*q*(Ixx-Iyy) + Mz)/Izz = Jz*p*q + Mz_
    
    # where    
    # Mx_ = k_p1*omega_1**2 + k_p2*omega_2**2 + k_p3*omega_3**2 + k_p4*omega_4**2
    # My_ = k_q1*omega_1**2 + k_q2*omega_2**2 + k_q3*omega_3**2 + k_q4*omega_4**2
    # Mz_ = k_r1*omega_1 + k_r2*omega_2 + k_r3*omega_3 + k_r4*omega_4 + k_r5*d_omega_1 + k_r6*d_omega_2 + k_r7*d_omega_3 + k_r8*d_omega_4
    
    # to get the derivative we use a low pass filter
    cutoff = 32 # Hz
    sos = sp.signal.butter(2, cutoff, 'low', fs=1/np.mean(np.diff(data['t'])), output='sos')
    
    dp = sp.signal.sosfiltfilt(sos, np.gradient(data['p'])/np.gradient(data['t']))
    dq = sp.signal.sosfiltfilt(sos, np.gradient(data['q'])/np.gradient(data['t']))
    dr = sp.signal.sosfiltfilt(sos, np.gradient(data['r'])/np.gradient(data['t']))
    
    domega_1 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[0]'])/np.gradient(data['t']))
    domega_2 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[1]'])/np.gradient(data['t']))
    domega_3 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[2]'])/np.gradient(data['t']))
    domega_4 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[3]'])/np.gradient(data['t']))
    
    params = np.load('params/aggressive_cmds2.npz')
    
    fig, axs = plt.subplots(3, 2, figsize=(10, 10), sharex=True, sharey=True)
    
    X = np.stack([
        data['omega[0]']**2,
        data['omega[1]']**2,
        data['omega[2]']**2,
        data['omega[3]']**2,
    ])
    Y = dp
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    k_p1, k_p2, k_p3, k_p4 = A
    dp_fit = A@X
    # high and low bounds based on r% variation per parameter
    A_low = A.copy()
    # the negative values in A should be multiplied by 1.3, the positive by 0.7
    A_low[A_low>0] *= 0.7
    A_low[A_low<0] *= 1.3
    A_high = A.copy()
    A_high[A_high>0] *= 1.3
    A_high[A_high<0] *= 0.7
    
    axs[0,0].plot(data['t'], Y, label='dp')
    axs[0,0].plot(data['t'], dp_fit, label='dp fit')
    axs[0,0].fill_between(data['t'], A_low@X, A_high@X, alpha=0.2, color='orange')
    axs[0,0].set_xlabel('t [s]')
    axs[0,0].set_ylabel('dp [rad/s^2]')
    axs[0,0].legend()
    axs[0,0].set_title('dp = k_p1*w1**2 + k_p2*w2**2 + k_p3*w3**2 + k_p4*w4**2 \n k_p1, k_p2, k_p3, k_p4 = {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A))
    
    X = np.stack([
        data['omega[0]']**2,
        data['omega[1]']**2,
        data['omega[2]']**2,
        data['omega[3]']**2,
    ])
    Y = dq
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    k_q1, k_q2, k_q3, k_q4 = A
    dq_fit = A@X
    # high and low bounds based on r% variation per parameter
    A_low = A.copy()
    # the negative values in A should be multiplied by 1.3, the positive by 0.7
    A_low[A_low>0] *= 0.7
    A_low[A_low<0] *= 1.3
    A_high = A.copy()
    A_high[A_high>0] *= 1.3
    A_high[A_high<0] *= 0.7 
    
    
    axs[1,0].plot(data['t'], Y, label='dq')
    axs[1,0].plot(data['t'], dq_fit, label='dq fit')
    axs[1,0].fill_between(data['t'], A_low@X, A_high@X, alpha=0.2, color='orange')
    axs[1,0].set_xlabel('t [s]')
    axs[1,0].set_ylabel('dq [rad/s^2]')
    axs[1,0].legend()
    axs[1,0].set_title('dq = k_q1*w1**2 + k_q2*w2**2 + k_q3*w3**2 + k_q4*w4**2 \n k_q1, k_q2, k_q3, k_q4 = {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A))
    
    
    X = np.stack([
        data['omega[0]'],
        data['omega[1]'],
        data['omega[2]'],
        data['omega[3]'],
        domega_1,
        domega_2,
        domega_3,
        domega_4,
        data['vbx']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]']),
        data['vby']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]']),
        data['vbz']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]']),
    ])
    Y = dr
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8, kvx, kvy, kvz = A
    dr_fit = A@X
    
    # X = np.stack([
    #     -data['omega[0]']+data['omega[1]']+data['omega[2]']-data['omega[3]'],
    #     -domega_1+domega_2+domega_3-domega_4,
    # ])
    # Y = dr
    # A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    # k_r, k_rd = A
    # dr_fit = A@X
    # high and low bounds based on r% variation per parameter
    A_low = A.copy()
    # the negative values in A should be multiplied by 1.3, the positive by 0.7
    A_low[A_low>0] *= 0.7
    A_low[A_low<0] *= 1.3
    A_high = A.copy()
    A_high[A_high>0] *= 1.3
    A_high[A_high<0] *= 0.7

     
    axs[2,0].plot(data['t'], Y, label='dr')
    # axs[2,0].plot(data['t'], dr_fit, label='dr fit')
    axs[2,0].plot(data['t'], dr_fit, label='dr fit')
    axs[2,0].fill_between(data['t'], A_low@X, A_high@X, alpha=0.2, color='orange')
    axs[2,0].set_xlabel('t [s]')
    axs[2,0].set_ylabel('dr [rad/s^2]')
    axs[2,0].legend()
    title = 'dr = k_r1*w1 + k_r2*w2 + k_r3*w3 + k_r4*w4 + k_r5*dw1 + k_r6*dw2 + k_r7*dw3 + k_r8*dw4 \n k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8 = {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A)
    # title = 'dr = k_r*(w1-w2+w3-w4) + k_rd*(dw1-dw2+dw3-dw4) \n k_r, k_rd = {:.2e}, {:.2e}'.format(*A)
    axs[2,0].set_title(title)
    
    # 3 plots with p,q,r
    axs[0,1].plot(data['t'], dp_fit-dp, label='dp fit error')
    axs[0,1].set_xlabel('t [s]')
    axs[0,1].set_ylabel('p [rad/s]')
    axs[0,1].legend()
    
    axs[1,1].plot(data['t'], dq_fit-dq, label='dq fit error')
    axs[1,1].set_xlabel('t [s]')
    axs[1,1].set_ylabel('q [rad/s]')
    axs[1,1].legend()
    
    axs[2,1].plot(data['t'], dr_fit-dr, label='dr fit error')
    axs[2,1].set_xlabel('t [s]')
    axs[2,1].set_ylabel('r [rad/s]')
    axs[2,1].legend()
    
    # show fig with the window name 'Moments Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Moments Model')
    plt.show()
    
    # print the results
    # print('k_p1, k_p2, k_p3, k_p4 = {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(k_p1, k_p2, k_p3, k_p4))
    # print('k_q1, k_q2, k_q3, k_q4 = {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(k_q1, k_q2, k_q3, k_q4))
    # print('k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8 = {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8))
    return k_p1, k_p2, k_p3, k_p4, k_q1, k_q2, k_q3, k_q4, k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8

def fit_inertia_model(data):
    print('fitting inertia model')
    print('assuming data is from a rotating free fall')
    # euler equations with no external torques
    # d_p     = q*r*(Iyy-Izz)/Ixx = Jx*q*r
    # d_q     = p*r*(Izz-Ixx)/Iyy = Jy*p*r
    # d_r     = p*q*(Ixx-Iyy)/Izz = Jz*p*q
    
    # to get the derivative we use a low pass filter
    cutoff = 32 # Hz
    sos = sp.signal.butter(2, cutoff, 'low', fs=1/np.mean(np.diff(data['t'])), output='sos')
    
    dp = sp.signal.sosfiltfilt(sos, np.gradient(data['p'])/np.gradient(data['t']))
    dq = sp.signal.sosfiltfilt(sos, np.gradient(data['q'])/np.gradient(data['t']))
    dr = sp.signal.sosfiltfilt(sos, np.gradient(data['r'])/np.gradient(data['t']))
    
    fig, axs = plt.subplots(3, 2, figsize=(10, 10), sharex=True, sharey=True)
    
    X = np.stack([data['q']*data['r']])
    Y = dp
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    Jx = A[0]
    dp_fit = A@X
    axs[0,0].plot(data['t'], Y, label='dp')
    axs[0,0].plot(data['t'], dp_fit, label='dp fit')
    axs[0,0].set_xlabel('t [s]')
    axs[0,0].set_ylabel('dp [rad/s^2]')
    axs[0,0].legend()
    axs[0,0].set_title('dp =  q*r*(Iyy-Izz)/Ixx  \n Jx = (Iyy-Izz)/Ixx = {:.2e}'.format(Jx))
    
    X = np.stack([data['p']*data['r']])
    Y = dq
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    Jy = A[0]
    dq_fit = A@X      
    axs[1,0].plot(data['t'], Y, label='dq')
    axs[1,0].plot(data['t'], dq_fit, label='dq fit')
    axs[1,0].set_xlabel('t [s]')
    axs[1,0].set_ylabel('dq [rad/s^2]')
    axs[1,0].legend()
    axs[1,0].set_title('dq = p*r*(Izz-Ixx)/Iyy \n Jy = (Izz-Ixx)/Iyy = {:.2e}'.format(Jy))
    
    X = np.stack([data['p']*data['q']])
    Y = dr
    A = np.linalg.lstsq(X.T, Y, rcond=None)[0]
    Jz = A[0]
    dr_fit = A@X
     
    axs[2,0].plot(data['t'], Y, label='dr')
    axs[2,0].plot(data['t'], dr_fit, label='dr fit')
    axs[2,0].set_xlabel('t [s]')
    axs[2,0].set_ylabel('dr [rad/s^2]')
    axs[2,0].legend()
    axs[2,0].set_title('dr = p*q*(Ixx-Iyy)/Izz \n Jz = (Ixx-Iyy)/Izz = {:.2e}'.format(Jz))
    
    # 3 plots with p,q,r
    axs[0,1].plot(data['t'], dp_fit-dp, label='dp fit error')
    axs[0,1].set_xlabel('t [s]')
    axs[0,1].set_ylabel('p [rad/s]')
    axs[0,1].legend()
    
    axs[1,1].plot(data['t'], dq_fit-dq, label='dq fit error')
    axs[1,1].set_xlabel('t [s]')
    axs[1,1].set_ylabel('q [rad/s]')
    axs[1,1].legend()
    
    axs[2,1].plot(data['t'], dr_fit-dr, label='dr fit error')
    axs[2,1].set_xlabel('t [s]')
    axs[2,1].set_ylabel('r [rad/s]')
    axs[2,1].legend()
    
    # show fig with the window name 'Moments Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Moments Model')
    plt.show()
    
    return Jx, Jy, Jz

def fit_imu_offset(data):
    print('fitting imu offset')
    print('assuming data is from a rotating free fall')
    
    # acceleration due to rotation model (imu offset rx, ry, rz)
    # ax = -rx*(wy*wy + wz*wz) + ry*(wx*wy - dwz) + rz*(wx*wz + dwy)
    # ay = rx*(wx*wy + dwz) - ry*(wx*wx + wz*wz) + rz*(wy*wz - dwx)
    # az = rx*(wx*wz - dwy) + ry*(wy*wz + dwx) - rz*(wx*wx + wy*wy)
    
    # to get the derivative we use a low pass filter
    cutoff = 32 # Hz
    sos = sp.signal.butter(2, cutoff, 'low', fs=1/np.mean(np.diff(data['t'])), output='sos')
    
    dp = sp.signal.sosfiltfilt(sos, np.gradient(data['p'])/np.gradient(data['t']))
    dq = sp.signal.sosfiltfilt(sos, np.gradient(data['q'])/np.gradient(data['t']))
    dr = sp.signal.sosfiltfilt(sos, np.gradient(data['r'])/np.gradient(data['t']))
    
    wx = data['p']
    wy = data['q']
    wz = data['r']
    
    dwx = dp
    dwy = dq
    dwz = dr
    

    # non-linear optimization
    def error(params):
        rx, ry, rz = params
        ax = -rx*(wy*wy + wz*wz) + ry*(wx*wy - dwz) + rz*(wx*wz + dwy)
        ay = rx*(wx*wy + dwz) - ry*(wx*wx + wz*wz) + rz*(wy*wz - dwx)
        az = rx*(wx*wz - dwy) + ry*(wy*wz + dwx) - rz*(wx*wx + wy*wy)
        return np.linalg.norm(ax - data['ax']) + np.linalg.norm(ay - data['ay']) + np.linalg.norm(az - data['az'])

    res = sp.optimize.minimize(error, [0, 0, 0])
    rx, ry, rz = res.x
    print('rx={:.5f}, ry={:.5f}, rz={:.5f}'.format(rx, ry, rz))
    
    ax_est = -rx*(wy*wy + wz*wz) + ry*(wx*wy - dwz) + rz*(wx*wz + dwy)
    ay_est = rx*(wx*wy + dwz) - ry*(wx*wx + wz*wz) + rz*(wy*wz - dwx)
    az_est = rx*(wx*wz - dwy) + ry*(wy*wz + dwx) - rz*(wx*wx + wy*wy)
    
    # plot
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True, sharey=True)

    axs[0].plot(data['t'], data['ax'], label='ax')
    axs[0].plot(data['t'], ax_est, label='ax est')
    axs[0].set_xlabel('t [s]')
    axs[0].set_ylabel('ax [m/s^2]')
    axs[0].legend()
    
    axs[1].plot(data['t'], data['ay'], label='ay')
    axs[1].plot(data['t'], ay_est, label='ay est')
    axs[1].set_xlabel('t [s]')
    axs[1].set_ylabel('ay [m/s^2]')
    axs[1].legend()
    
    axs[2].plot(data['t'], data['az'], label='az')
    axs[2].plot(data['t'], az_est, label='az est')
    axs[2].set_xlabel('t [s]')
    axs[2].set_ylabel('az [m/s^2]')
    axs[2].legend()
    
    # show fig with the window name 'IMU Offset'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('IMU Offset')
    plt.show()
    
    
    
    

def plot_moments_model(data, params, randomization=30):
    print('fitting moments model')
    # model from https://doi.org/10.1016/j.robot.2023.104588
    # d_p     = (q*r*(Iyy-Izz) + Mx)/Ixx = Jx*q*r + Mx_
    # d_q     = (p*r*(Izz-Ixx) + My)/Iyy = Jy*p*r + My_
    # d_r     = (p*q*(Ixx-Iyy) + Mz)/Izz = Jz*p*q + Mz_
    
    # where    
    # Mx_ = k_p1*omega_1**2 + k_p2*omega_2**2 + k_p3*omega_3**2 + k_p4*omega_4**2
    # My_ = k_q1*omega_1**2 + k_q2*omega_2**2 + k_q3*omega_3**2 + k_q4*omega_4**2
    # Mz_ = k_r1*omega_1 + k_r2*omega_2 + k_r3*omega_3 + k_r4*omega_4 + k_r5*d_omega_1 + k_r6*d_omega_2 + k_r7*d_omega_3 + k_r8*d_omega_4
    
    # to get the derivative we use a low pass filter
    cutoff = 32 # Hz
    sos = sp.signal.butter(2, cutoff, 'low', fs=1/np.mean(np.diff(data['t'])), output='sos')
    
    dp = sp.signal.sosfiltfilt(sos, np.gradient(data['p'])/np.gradient(data['t']))
    dq = sp.signal.sosfiltfilt(sos, np.gradient(data['q'])/np.gradient(data['t']))
    dr = sp.signal.sosfiltfilt(sos, np.gradient(data['r'])/np.gradient(data['t']))
    
    domega_1 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[0]'])/np.gradient(data['t']))
    domega_2 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[1]'])/np.gradient(data['t']))
    domega_3 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[2]'])/np.gradient(data['t']))
    domega_4 = sp.signal.sosfiltfilt(sos, np.gradient(data['omega[3]'])/np.gradient(data['t']))
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True, sharey=True)
    
    X = np.stack([
        data['omega[0]']**2,
        data['omega[1]']**2,
        data['omega[2]']**2,
        data['omega[3]']**2,
        data['q']*data['r'],
    ])
    Y = dp
    # get A from the params
    A = np.array([params['k_p1'], params['k_p2'], params['k_p3'], params['k_p4'], params['Jx']])
    k_p1, k_p2, k_p3, k_p4, Jx = A
    A_ = A.copy()
    A_[-1] = 0
    dp_fit = A_@X
    dp_fit_gyro = A@X
    # 1000 fits with r% variation in parameters
    r = randomization/100
    dq_fits = np.zeros((1000, len(data['t'])))
    dq_fits_gyro = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        # generate random parameters
        Arandom = np.random.uniform(1-r, 1+r, len(A_))*A_
        dq_fits[i] = Arandom@X
        Arandom = np.random.uniform(1-r, 1+r, len(A))*A
        dq_fits_gyro[i] = Arandom@X
    dq_fit_high = np.max(dq_fits, axis=0)
    dq_fit_low = np.min(dq_fits, axis=0)
    dq_fit_high_gyro = np.max(dq_fits_gyro, axis=0)
    dq_fit_low_gyro = np.min(dq_fits_gyro, axis=0)
        
    axs[0].plot(data['t'], Y, label='dp')
    axs[0].plot(data['t'], dp_fit, label='dp fit')
    axs[0].fill_between(data['t'], dq_fit_low, dq_fit_high, alpha=0.2, color='orange')
    axs[0].plot(data['t'], dp_fit_gyro, label='dp fit gyro')
    axs[0].fill_between(data['t'], dq_fit_low_gyro, dq_fit_high_gyro, alpha=0.2, color='green')
    axs[0].set_xlabel('t [s]')
    axs[0].set_ylabel('dp [rad/s^2]')
    axs[0].legend()
    axs[0].set_title('dp = k_p1*w1**2 + k_p2*w2**2 + k_p3*w3**2 + k_p4*w4**2 + Jx*q*r \n k_p1, k_p2, k_p3, k_p4, Jx = {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A))
    
    X = np.stack([
        data['omega[0]']**2,
        data['omega[1]']**2,
        data['omega[2]']**2,
        data['omega[3]']**2,
        data['p']*data['r'],
    ])
    Y = dq
    # get A from the params
    A = np.array([params['k_q1'], params['k_q2'], params['k_q3'], params['k_q4'], params['Jy']])
    k_q1, k_q2, k_q3, k_q4, Jy = A
    A_ = A.copy()
    A_[-1] = 0
    dq_fit = A_@X
    dq_fit_gyro = A@X
    # 1000 fits with r% variation in parameters
    r = randomization/100
    dq_fits = np.zeros((1000, len(data['t'])))
    dq_fits_gyro = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        # generate random parameters
        Arandom = np.random.uniform(1-r, 1+r, len(A_))*A_
        dq_fits[i] = Arandom@X
        Arandom = np.random.uniform(1-r, 1+r, len(A))*A
        dq_fits_gyro[i] = Arandom@X
    dq_fit_high = np.max(dq_fits, axis=0)
    dq_fit_low = np.min(dq_fits, axis=0)
    dq_fit_high_gyro = np.max(dq_fits_gyro, axis=0)
    dq_fit_low_gyro = np.min(dq_fits_gyro, axis=0)                            
    
    axs[1].plot(data['t'], Y, label='dq')
    axs[1].plot(data['t'], dq_fit, label='dq fit')
    axs[1].fill_between(data['t'], dq_fit_low, dq_fit_high, alpha=0.2, color='orange')
    axs[1].plot(data['t'], dq_fit_gyro, label='dq fit gyro')
    axs[1].fill_between(data['t'], dq_fit_low_gyro, dq_fit_high_gyro, alpha=0.2, color='green')
    axs[1].set_xlabel('t [s]')
    axs[1].set_ylabel('dq [rad/s^2]')
    axs[1].legend()
    axs[1].set_title('dq = k_q1*w1**2 + k_q2*w2**2 + k_q3*w3**2 + k_q4*w4**2 + Jy*p*r \n k_q1, k_q2, k_q3, k_q4, Jy = {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A))
    
    
    X = np.stack([
        data['omega[0]'],
        data['omega[1]'],
        data['omega[2]'],
        data['omega[3]'],
        domega_1,
        domega_2,
        domega_3,
        domega_4,
        data['p']*data['q'] #params['k_y']*data['vby']*(data['omega[0]']+data['omega[1]']+data['omega[2]']+data['omega[3]']),
    ])
    Y = dr
    # get A from the params
    A = np.array([params['k_r1'], params['k_r2'], params['k_r3'], params['k_r4'], params['k_r5'], params['k_r6'], params['k_r7'], params['k_r8'], params['Jz']])
    k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8, Jz = A
    A_ = A.copy()
    A_[-1] = 0
    dr_fit = A_@X
    dr_fit_gyro = A@X
    # 1000 fits with r% variation in parameters
    r = randomization/100
    dr_fits = np.zeros((1000, len(data['t'])))
    dr_fits_gyro = np.zeros((1000, len(data['t'])))
    for i in range(1000):
        # generate random parameters
        Arandom = np.random.uniform(1-r, 1+r, len(A_))*A_
        dr_fits[i] = Arandom@X
        Arandom = np.random.uniform(1-r, 1+r, len(A))*A
        dr_fits_gyro[i] = Arandom@X
    dr_fit_high = np.max(dr_fits, axis=0)
    dr_fit_low = np.min(dr_fits, axis=0)
    dr_fit_high_gyro = np.max(dr_fits_gyro, axis=0)
    dr_fit_low_gyro = np.min(dr_fits_gyro, axis=0)
     
    axs[2].plot(data['t'], Y, label='dr')
    axs[2].plot(data['t'], dr_fit, label='dr fit')
    axs[2].fill_between(data['t'], dr_fit_low, dr_fit_high, alpha=0.2, color='orange')
    axs[2].plot(data['t'], dr_fit_gyro, label='dr fit gyro')
    axs[2].fill_between(data['t'], dr_fit_low_gyro, dr_fit_high_gyro, alpha=0.2, color='green')
    axs[2].set_xlabel('t [s]')
    axs[2].set_ylabel('dr [rad/s^2]')
    axs[2].legend()
    title = 'dr = k_r1*w1 + k_r2*w2 + k_r3*w3 + k_r4*w4 + k_r5*dw1 + k_r6*dw2 + k_r7*dw3 + k_r8*dw4 + Jz*p*q \n k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8, Jz = {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}, {:.2e}'.format(*A)
    # title = 'dr = k_r*(w1-w2+w3-w4) + k_rd*(dw1-dw2+dw3-dw4) \n k_r, k_rd = {:.2e}, {:.2e}'.format(*A)
    axs[2].set_title(title)
    
    # # 3 plots with p,q,r
    # axs[0,1].plot(data['t'], dp_fit-dp, label='dp fit error')
    # axs[0,1].set_xlabel('t [s]')
    # axs[0,1].set_ylabel('p [rad/s]')
    # axs[0,1].legend()
    
    # axs[1,1].plot(data['t'], dq_fit-dq, label='dq fit error')
    # axs[1,1].set_xlabel('t [s]')
    # axs[1,1].set_ylabel('q [rad/s]')
    # axs[1,1].legend()
    
    # axs[2,1].plot(data['t'], dr_fit-dr, label='dr fit error')
    # axs[2,1].set_xlabel('t [s]')
    # axs[2,1].set_ylabel('r [rad/s]')
    # axs[2,1].legend()
    
    # show fig with the window name 'Moments Model'
    manager = plt.get_current_fig_manager()
    manager.set_window_title('Moments Model')
    
    # set all y axis lims to +- 100
    # for ax in axs.flatten():
    #     ax.set_ylim([-100, 100])
    plt.show()
    return k_p1, k_p2, k_p3, k_p4, k_q1, k_q2, k_q3, k_q4, k_r1, k_r2, k_r3, k_r4, k_r5, k_r6, k_r7, k_r8




# FUNCTIONS FOR PLOTTING
def ekf_plot(data):
    ekf_updates = (np.diff(data['x_opti']) != 0) | (np.diff(data['y_opti']) != 0) | (np.diff(data['z_opti']) != 0) | (np.diff(data['phi_opti']) != 0) | (np.diff(data['theta_opti']) != 0) | (np.diff(data['psi_opti']) != 0)
    ekf_updates = np.where(ekf_updates)[0]+1
    # set to all indices
    ekf_updates = np.arange(len(data['t']))
    
    # # generate opti vx,vy,vz
    # vx_opti = np.gradient(data['x_opti'][ekf_updates], data['t'][ekf_updates])
    # vy_opti = np.gradient(data['y_opti'][ekf_updates], data['t'][ekf_updates])
    # vz_opti = np.gradient(data['z_opti'][ekf_updates], data['t'][ekf_updates])
    
    # subplots 1x3 with ekf_x, ekf_y, ekf_z
    fig, axs = plt.subplots(5, 3, figsize=(10,10), sharex=True, sharey='row', tight_layout=True)

    # POSITION
    plt.sca(axs[0,0])
    plt.plot(data['t'], data['ekf_x'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['x_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('x [m]')
    plt.legend()
    plt.sca(axs[0,1])
    plt.plot(data['t'], data['ekf_y'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['y_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.sca(axs[0,2])
    plt.plot(data['t'], data['ekf_z'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['z_opti'][ekf_updates], label='opti')
    # plot line at zero
    plt.plot(data['t'], data['t']*0)
    plt.xlabel('t [s]')
    plt.ylabel('z [m]')
    plt.legend()

    # VELOCITY
    plt.sca(axs[1,0])
    plt.plot(data['t'], data['ekf_vx'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['vx_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('vx [m/s]')
    plt.legend()
    plt.sca(axs[1,1])
    plt.plot(data['t'], data['ekf_vy'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['vy_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('vy [m/s]')
    plt.legend()
    plt.sca(axs[1,2])
    plt.plot(data['t'], data['ekf_vz'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['vz_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('vz [m/s]')
    plt.legend()

    # ATTITUDE
    plt.sca(axs[2,0])
    plt.plot(data['t'], data['ekf_phi'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['phi_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('phi [rad]')
    plt.legend()
    plt.sca(axs[2,1])
    plt.plot(data['t'], data['ekf_theta'], label='ekf')
    plt.plot(data['t'][ekf_updates], data['theta_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('theta [rad]')
    plt.legend()
    plt.sca(axs[2,2])
    ekf_psi = ((data['ekf_psi']+np.pi)%(2*np.pi))-np.pi
    plt.plot(data['t'], ekf_psi, label='ekf')
    plt.plot(data['t'][ekf_updates], data['psi_opti'][ekf_updates], label='opti')
    plt.xlabel('t [s]')
    plt.ylabel('psi [rad]')
    plt.legend()

    # ACC BIAS
    plt.sca(axs[3,0])
    plt.plot(data['t'], data['ekf_acc_b_x'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('acc_b_x [m/s^2]')
    plt.legend()
    plt.sca(axs[3,1])
    plt.plot(data['t'], data['ekf_acc_b_y'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('acc_b_y [m/s^2]')
    plt.legend()
    plt.sca(axs[3,2])
    plt.plot(data['t'], data['ekf_acc_b_z'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('acc_b_z [m/s^2]')
    plt.legend()

    # GYRO BIAS
    plt.sca(axs[4,0])
    plt.plot(data['t'], data['ekf_gyro_b_x'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('gyro_b_x [rad/s]')
    plt.legend()
    plt.sca(axs[4,1])
    plt.plot(data['t'], data['ekf_gyro_b_y'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('gyro_b_y [rad/s]')
    plt.legend()
    plt.sca(axs[4,2])
    plt.plot(data['t'], data['ekf_gyro_b_z'], label='ekf')
    plt.xlabel('t [s]')
    plt.ylabel('gyro_b_z [rad/s]')
    plt.legend()

    plt.show()
    
def acc_plot(data):
    # plot imu measurements
    fig, axs = plt.subplots(1, 3, figsize=(10,5), sharex=True, sharey='col', tight_layout=True)

    # ACCELEROMETER
    # X
    plt.sca(axs[0])
    if 'ax_unfiltered' in data:
        plt.plot(data['t'], data['ax_unfiltered'], label='ax raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['ax'], label='ax')
    # plt.plot(data['t'], data['ax_filt'], label='ax_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('ax [m/s^2]')
    plt.legend()
    # Y
    plt.sca(axs[1])
    if 'ay_unfiltered' in data:
        plt.plot(data['t'], data['ay_unfiltered'], label='ay raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['ay'], label='ay')
    # plt.plot(data['t'], data['ay_filt'], label='ay_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('ay [m/s^2]')
    plt.legend()
    # Z
    plt.sca(axs[2])
    if 'az_unfiltered' in data:
        plt.plot(data['t'], data['az_unfiltered'], label='az raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['az'], label='az')
    # plt.plot(data['t'], data['az_filt'], label='az_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('az [m/s^2]')
    plt.legend()

    plt.show()
    
def imu_plot(data, **kwargs):
    # plot imu measurements
    fig, axs = plt.subplots(2, 3, figsize=(10,5), sharex=True, sharey='row', tight_layout=True)

    # ACCELEROMETER
    # X
    plt.sca(axs[0,0])
    if 'ax_unfiltered' in data:
        plt.plot(data['t'], data['ax_unfiltered'], label='ax raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['ax'], label='ax')
    # plt.plot(data['t'], data['ax_filt'], label='ax_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('ax [m/s^2]')
    plt.legend()
    # Y
    plt.sca(axs[0,1])
    if 'ay_unfiltered' in data:
        plt.plot(data['t'], data['ay_unfiltered'], label='ay raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['ay'], label='ay')
    # plt.plot(data['t'], data['ay_filt'], label='ay_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('ay [m/s^2]')
    plt.legend()
    # Z
    plt.sca(axs[0,2])
    if 'az_unfiltered' in data:
        plt.plot(data['t'], data['az_unfiltered'], label='az raw', alpha=0.5, color='blue')
    plt.plot(data['t'], data['az'], label='az')
    if 'k_w' in kwargs:
        plt.plot(data['t'], -kwargs['k_w']*(data['omega[0]']**2+data['omega[1]']**2+data['omega[2]']**2+data['omega[3]']**2), label='thrust model')
    # plt.plot(data['t'], data['az_filt'], label='az_filt')
    plt.ylim([-160,160])
    plt.xlabel('t [s]')
    plt.ylabel('az [m/s^2]')
    plt.legend()
    
    # GYROSCOPE
    # X
    plt.sca(axs[1,0])
    plt.plot(data['t'], data['p'], label='p')
    plt.xlabel('t [s]')
    plt.ylabel('p [rad/s]')
    plt.legend()
    # Y
    plt.sca(axs[1,1])
    plt.plot(data['t'], data['q'], label='q')
    plt.xlabel('t [s]')
    plt.ylabel('q [rad/s]')
    plt.legend()
    # Z
    plt.sca(axs[1,2])
    plt.plot(data['t'], data['r'], label='r')
    plt.xlabel('t [s]')
    plt.ylabel('r [rad/s]')
    plt.legend()

    plt.show()
    
    
def actuator_plot(data):
    # 4x2 subplots with u, omega
    fig, axs = plt.subplots(4, 2, figsize=(5,5), sharex=True, sharey='col', tight_layout=True)
    
    # MOTOR COMMANDS
    # 1
    plt.sca(axs[0,0])
    plt.plot(data['t'], data['u1'], label='u1')
    plt.xlabel('t [s]')
    plt.ylabel('u1')
    plt.legend()
    # 2
    plt.sca(axs[1,0])
    plt.plot(data['t'], data['u2'], label='u2')
    plt.xlabel('t [s]')
    plt.ylabel('u2')
    plt.legend()
    # 3
    plt.sca(axs[2,0])
    plt.plot(data['t'], data['u3'], label='u3')
    plt.xlabel('t [s]')
    plt.ylabel('u3')
    plt.legend()
    # 4
    plt.sca(axs[3,0])
    plt.plot(data['t'], data['u4'], label='u4')
    plt.xlabel('t [s]')
    plt.ylabel('u4')
    plt.legend()
    
    # MOTOR SPEED
    # 1
    plt.sca(axs[0,1])
    plt.plot(data['t'], data['omega[0]'], label='omega1')
    plt.xlabel('t [s]')
    plt.ylabel('omega1 [rad/s]')
    plt.legend()
    # 2
    plt.sca(axs[1,1])
    plt.plot(data['t'], data['omega[1]'], label='omega2')
    plt.xlabel('t [s]')
    plt.ylabel('omega2 [rad/s]')
    plt.legend()
    # 3
    plt.sca(axs[2,1])
    plt.plot(data['t'], data['omega[2]'], label='omega3')
    plt.xlabel('t [s]')
    plt.ylabel('omega3 [rad/s]')
    plt.legend()
    # 4
    plt.sca(axs[3,1])
    plt.plot(data['t'], data['omega[3]'], label='omega4')
    plt.xlabel('t [s]')
    plt.ylabel('omega4 [rad/s]')
    plt.legend()
    
    plt.show()
    
    
def xy_plot(data, **kwargs):
    # figure with a xy plot of 10x10m that shows the drone trajectory + gates (if provided)
    plt.figure(figsize=(10,10))
    # if 'x' in data:
    #     plt.plot(data['x'], data['y'], label='est')
    # if 'ekf_x' in data:
    #     plt.plot(data['ekf_x'], data['ekf_y'], label='ekf')
    if 'x_opti' in data:
        plt.plot(data['x_opti'], data['y_opti'], label='opti')
    # gate gate_pos and gate_yaw form the kwargs
    if 'gate_pos' in kwargs and 'gate_yaw' in kwargs:
        for i in range(len(kwargs['gate_pos'])):
            x, y, z = kwargs['gate_pos'][i]
            yaw = kwargs['gate_yaw'][i]
            plt.plot([x-np.sin(yaw)*0.75, x+np.sin(yaw)*0.75], [y-np.cos(yaw)*0.75, y+np.cos(yaw)*0.75], color='red')
    plt.xlim([-4, 4])
    plt.ylim([-4, 4])
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.show()