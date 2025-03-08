clc, clearvars

% using minimal longitudinal model data from the science paper;

load('torqueCoupling.mat', 'experiment102');

data = experiment102;

% I will be using pitch 15 to validate the model in:
% "Minimal longitudinal dynamic model"

% time data
time_optitrack = data.motion_tracking.TIME;
time0_optitrack = find(time_optitrack == 0);

time_onboard = data.onboard.angles_commands_setpoints.TIME_onboard{2, 1}; % to fix
time_onboard_freq = data.onboard.frequency.TIME_onboard_freq{2, 1}; % to fix

% this dihedral value is the value from the optitrack, contains various NaN
dihedral_avg = data.motion_tracking.DIHEDRAL_avg;
dihedral_std = data.motion_tracking.DIHEDRAL_std;

% unfiltered acceleration in the u direction (body x) and w direction (body
% z) and thetadd, this is what we need to predict
ud = data.motion_tracking.ACC_BODYx_avg;
wd = data.motion_tracking.ACC_BODYz_avg;
thetadd = data.motion_tracking.ALPHy_avg;


% Initial Conditions
u = data.motion_tracking.VEL_BODYx_avg;
w = data.motion_tracking.VEL_BODYy_avg;
thetad = data.motion_tracking.OMy_avg;
theta = data.motion_tracking.PITCH_avg;


pprz_cmd = data.onboard.angles_commands_setpoints.CMDpitch_filtered{2, 1} * 96 ; % to fix
freq_R = data.onboard.frequency.FREQright_wing{2, 1}/60; % to fix
const_f0 = mean(freq_R, 'all');

% Filter now with a 4th order butterworth 5Hz cut-off to remove noise
% due to flapping

fs = 200; %Hz
fc = 5; % Hz

order = 4;

[b, a] = butter(order, fc/(fs/2), 'low'); 

% filter acc and thetadd
ud_f = filtfilt(b, a, ud);
wd_f = filtfilt(b, a, wd);
thetadd_f = filtfilt(b, a, thetadd);

% filter for initial conditions
u_f =  filtfilt(b, a, u);
w_f = filtfilt(b, a, w);
thetad_f =filtfilt(b, a, thetad);
theta_f = filtfilt(b, a, theta);

% choose initial conditions
u_f0 = u_f(time0_optitrack);
w_f0 = w_f(time0_optitrack);
thetad_f0 = thetad_f(time0_optitrack);
theta_f0 = theta_f(time0_optitrack);

% finally figured out the data (01/03) : our inputs to the model are pprz_cmd and
% const_f0: assuming the frequency stays constant at all phases (maneuver
% IS at constant thrust)

% now try to run the model (scary)

model = 'OL_fullnonlin_prevval_ucorr';
load_system(model);

% setting up the parameters
pars.f0 = const_f0; % Hz
pars.m = 29.4e-3; % kg
pars.I = 1e-4; % kg m^2
pars.c1 = 0.0114; % [-]
pars.c2 = -0.0449; %[-]
pars.bx = 0.0722; % N s^-1 m^-1
pars.lz = 0.0271; % m 
pars.ly = 0.081;
pars.bz = 0.0157;
pars.lx = 0;

% actuator dynamics dihedral
pars.act_w0 = 40;
pars.act_damp = 0.634;
pars.ucorr = 10;

% set initial conditions
pars.w0 = w_f0;
pars.thetad0 = thetad_f0*pi/180;
pars.theta0 = theta_f0*pi/180;
pars.u0 = u_f0;

% confusion starts here

% prepare input data
% time 
t_dat = time_onboard;
cmdpprz_dat = pprz_cmd;

input_data = Simulink.SimulationData.Dataset();
input_data = input_data.addElement([t_dat cmdpprz_dat],'pprz_cmd');

save('inputdata.mat', 'input_data')

cell_input = {'input_data.getElement(1)'};

cellOfErrors = cell(1,length( cell_input ));
simOut = Simulink.SimulationOutput.empty(0,length( cell_input ));


dataarray = input_data.get(1);
stoptime = dataarray(end,1);

simOut(1) = sim( 'OL_fullnonlin_prevval_ucorr.slx', 'ExternalInput', cell_input{1}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);

% it finally run 02/03/2025
% i should clean the workspace variables but that's for another time
udot_sim = simOut(1).yout{1};
udot_sim_t = udot_sim.Values.Time;
udot_sim_value = udot_sim.Values.Data;

dih_angle_sim = simOut(1).yout{8};
dih_angle_sim_t = dih_angle_sim.Values.Time;
dih_angle_sim_value = dih_angle_sim.Values.Data;

% data is clearly not right

% PLOTTING
tiledlayout(7, 1)

nexttile


plot(time_optitrack, ud_f)

hold on
plot(udot_sim_t, udot_sim_value, '--')
xlim([0 2.5])
ylim([ -1 5])
ylabel('$\dot{u} [m/s^2]$', 'Interpreter','latex')

hold off

nexttile
plot(time_optitrack, wd_f)
xlim([0 2.5])
ylabel('$\dot{w} [m/s^2]$', 'Interpreter','latex')

nexttile

plot(time_optitrack, dihedral_avg)
hold on
plot(dih_angle_sim_t, dih_angle_sim_value, '--')
hold off
xlim([0 2.5])
ylabel('$\gamma_2 [deg]$', 'Interpreter','latex')
xlabel('Time [s]')

nexttile 
plot(time_optitrack, thetadd_f)
xlim([0 2.5])
ylabel('$\ddot{\theta} [rad/s^2]$', 'Interpreter','latex')
xlabel('Time [s]')

nexttile
plot(time_onboard, pprz_cmd)
xlim([0 2.5])
ylabel('PPRZ cmd', 'Interpreter','latex')
xlabel('Time [s]')

nexttile
plot(time_onboard_freq, freq_R)
xlim([0 2.5])
ylabel('freq right wing [Hz]', 'Interpreter','latex')
xlabel('Time [s]')

nexttile
plot(time_onboard, pprz_cmd)
xlim([0 2.5])