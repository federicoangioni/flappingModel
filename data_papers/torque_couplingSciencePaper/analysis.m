clc, clearvars

load flight_data_torque_couplingSciencePaper.mat;
% 93/103 pitch 45°
% 94 pitch 60 deg
% 101 pitch 30 deg
% 102 pitch 15 deg
% 104 pitch 75 deg

% 95 roll 15 deg
% 100 roll 30 deg
% 99 roll 45 deg
% 98 roll 60 deg

% load in the necessary data
pitch75 = experiment104.motion_tracking;
pitch60 = experiment104;
pitch45 = experiment103.motion_tracking;
pitch30 = experiment101.motion_tracking;
pitch15 = experiment102.motion_tracking;


% I will be using pitch 60 to validate the model in:
% "Minimal longitudinal dynamic model"

% time
time = pitch60.motion_tracking.TIME;

% this dihedral value is the value from the optitrack, contains various NaN
% (not useful)
dihedral_avg = pitch60.motion_tracking.DIHEDRAL_avg;
dihedral_std = pitch60.motion_tracking.DIHEDRAL_std;

% unfiltered acceleration in the u direction (body x) and w direction (body z)
u_dot = pitch60.motion_tracking.ACC_BODYx_avg;

w_dot = pitch60.motion_tracking.ACC_BODYz_avg;

% Filter now with a butterworth 4th order 5Hz

fs = 200; %Hz
fc = 5; % Hz

order = 4;

[b, a] = butter(order, fc/(fs/2), 'low'); 

u_dot_filtered = filtfilt(b, a, u_dot);
w_dot_filtered = filtfilt(b, a, w_dot);


% Plotting
tiledlayout(3, 1)

nexttile

plot(time, u_dot_filtered)
xlim([0 2.5])
ylabel('$\dot{u} [m/s^2]$', 'Interpreter','latex')

nexttile
plot(time, w_dot_filtered)
xlim([0 2.5])
ylabel('$\dot{w} [m/s^2]$', 'Interpreter','latex')

nexttile
plot(time, dihedral_avg)
xlim([0 2.5])
ylabel('$\gamma_2 [deg]$', 'Interpreter','latex')
xlabel('Time [s]')