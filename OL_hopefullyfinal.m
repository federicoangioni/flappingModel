clc; clearvars;

%% Trying to use the Science data finally

load('data\torqueCoupling.mat')

optitrack = experiment57.motion_tracking;

ud = optitrack.ACC_BODYx(1, :);

ud
fs = 200; %Hz
fc = 5; % Hz

order = 4;

[b, a] = butter(order, fc/(fs/2), 'low'); 

ud_f = filtfilt(b, a, ud);

plot(optitrack.TIME, ud_f);
xlim([0 2.5])
