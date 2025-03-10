clc; clearvars;

%% Trying to use the Science data finally

load('data\torqueCoupling.mat')
% 4 might be similar

optitrack = experiment94.motion_tracking;
onboard = experiment94.onboard;
sp_cmd = onboard.angles_commands_setpoints.CMDpitch{1};

ud = optitrack.ACC_BODYx_avg;
wd = optitrack.ACC_BODYz_avg;

thetadd_opti = optitrack.ALPHy_avg;
thetadd_imu = onboard.rates.ALPHy_IMU{1};

fs = 360; %Hz
fc = 5; % Hz

order = 2;

[b, a] = butter(order, fc/(fs/2), 'low'); 

ud_f = filtfilt(b, a, ud);
wd_f = filtfilt(b, a, wd);

thetadd_opti_f = filtfilt(b, a, thetadd_opti);


% different butterworth filter due to different sampling frequency
fs = 100; %Hz
fc = 5; % Hz

order = 4;

[d, c] = butter(order, fc/(fs/2), 'low'); 

thetadd_imu_f = filtfilt(d, c, thetadd_imu);


tiledlayout(5, 1)
nexttile
plot(optitrack.TIME, ud_f);
ylabel('$\dot{u} [m/s]$', Interpreter='latex')
xlim([0 2.5])

nexttile
plot(optitrack.TIME, wd_f);
ylabel('$\dot{w} [m/s]$', Interpreter='latex')
xlim([0 2.5])
ylim([-7.5 15])



nexttile
plot(onboard.rates.TIME_onboard_rates{1}, thetadd_imu_f*pi/180)
ylabel('$\ddot{\theta} [deg/s]$ onboard', Interpreter='latex')
xlim([0 2.5])
nexttile
plot(optitrack.TIME, thetadd_opti_f*pi/180)
ylabel('$\ddot{\theta} [deg/s]$ opti', Interpreter='latex')
xlim([0 2.5])

nexttile
plot(onboard.angles_commands_setpoints.TIME_onboard{1}, sp_cmd)
ylabel('sp_pitch $[deg/s]$ opti', Interpreter='latex')
xlim([0 2.5])

