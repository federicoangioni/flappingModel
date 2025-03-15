clc; clearvars;
%% Main comments
% Managed to get a similar curve, i think the issue lies now in the wrong 
% parameters, use the torquecoupling data to do this


% Running open loop after Karàsek email 
% load in the experiment with 15 deg of pitch
i = 1;

%% organise the input to the model
% model takes time and pitch paparazzi command, after Matej's consultation
% pitch_cmd should be multiplied by 96

load BAL15_set.mat;

onboard = testsids{1}.onboard;
optitrack = testsids{1}.opti;
%% info on the dataset
% set pitch is in degrees

sp_pitch = (onboard.thetacmd)*180/pi;
pprz_pitch = onboard.pitchcmdF'*96;

time = testsids{1}.t';

%% Butterworth filtering
% Filter now with a 4th order butterworth 5Hz cut-off to remove noise
% due to flapping

fs = 360; %Hz
fc = 5; % Hz

order = 4;

[b, a] = butter(order, fc/(fs/2), 'low'); 

pprz_filt = filtfilt(b, a, pprz_pitch);

%% Inputs
% prepare input data to be processed by the model
input_data = Simulink.SimulationData.Dataset();
input_data = input_data.addElement([time pprz_filt], 'PPRZ');

%% Model parameters
% obtained from paper "A minimal longitudinal model ..."
earliestsetpointstarttime = 9999999;
earliestendtime = 9999999;


findpts = testsids{1}.onboard.thetacmd/pi*180;
findpts(abs(findpts)/pi*180<10) = 0;
earliestind = find(findpts,1);
earliesttime = testsids{i}.t(earliestind);

testsids{1}.setpointstarttime = earliesttime;


cell_input = {'input_data.getElement(1)'};

dataarray = input_data.get(1);
stoptime = dataarray(end,1);
assignin('base','pars', testpars);
% parameters from paper

pars.I = 1.26e-4;
pars.lz = 0.035;
pars.ucorr = 10;
pars.f0 = 16.4164;
pars.m = 29.4e-3;

pars.f0 = testpars.f0;
pars.w0 =  0.1217;

cd('models')
simOut = sim('OL_fullnonlin_prevval_ucorr.slx', 'ExternalInput', cell_input{1}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);
cd('..')

yout = get(simOut,'yout');

%% Collect data from simulation

% Accelerations:
% u dot
sim_ud = simOut.yout{1}.Values.Data;
sim_ud_t = simOut.yout{1}.Values.Time;

% w dot
sim_wd = simOut.yout{3}.Values.Data;
sim_wd_t = simOut.yout{3}.Values.Time;

% theta double dot
sim_thetadd = simOut.yout{5}.Values.Data;
sim_thetadd_t = simOut.yout{5}.Values.Time;

% Velocities:
% u
sim_u = simOut.yout{2}.Values.Data;
sim_u_t = simOut.yout{2}.Values.Time;

% w
sim_w = simOut.yout{4}.Values.Data;
sim_w_t = simOut.yout{4}.Values.Time;

% thetad
sim_thetad = simOut.yout{6}.Values.Data;
sim_thetad_t = simOut.yout{6}.Values.Time;

% Pitch angle theta
sim_theta = simOut.yout{7}.Values.Data;
sim_theta_t = simOut.yout{7}.Values.Time;

% Dihedral angle
sim_dih = simOut.yout{8}.Values.Data;
sim_dih_t = simOut.yout{8}.Values.Time;

%% Plot
% Define common x limit
limitx = stoptime;

% First plot accelerations and command input
figure(Name= "Accelerations and input")
t = tiledlayout(4, 1);
title(t, 'Open Loop $\theta = 15 \deg$, accelerations', Interpreter='Latex')

nexttile()
plot(sim_ud_t, sim_ud, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.udFF, DisplayName= 'data');
ylabel('$\dot{u} \; [m/s]$', Interpreter='latex');
legend();

xlim([0 limitx]);
ylim([-5 5]);

nexttile();
plot(sim_wd_t, sim_wd, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.wdFF, DisplayName= 'data');
ylabel('$\dot{w} \; [m/s]$', Interpreter='latex');

xlim([0 limitx]);
ylim([-5 5]);

nexttile();
plot(sim_thetadd_t, sim_thetadd, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.thetaddFF, DisplayName='Data');
ylabel('$\ddot{\theta} \; [rad/s]$', Interpreter='latex');

xlim([0 limitx]);
ylim([-20 20]);


nexttile()
plot(input_data{1}(:, 1), pprz_filt);
ylabel('$ PPRZ \: cmd \: [\% \cdot 96] $', Interpreter='latex');
xlabel('$Time [s]$', Interpreter='latex');
xlim([0 limitx]);

saveas(gcf, 'figures/openLoop_accelerations_input.png')

figure(Name= "Velocities and Angular Velocity");
t = tiledlayout(3, 1);
title(t, 'Open Loop $\theta = 15 \deg$, velocities', Interpreter='Latex');

nexttile()
plot(sim_u_t, sim_u, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.uFF, DisplayName='Data');
ylabel('$u \; [m/s]$', Interpreter='latex');

xlim([0 limitx]);

nexttile();
plot(sim_w_t, sim_w, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.wFF, DisplayName= 'Data');
ylabel('$w \; [m/s]$', Interpreter='latex');

xlim([0 limitx]);

nexttile();
plot(sim_thetad_t, sim_thetad, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.thetadFF, DisplayName= 'Data');
ylabel('$\dot{\theta} \; [\deg/s]$', Interpreter='latex');
xlabel('$Time [s]$', Interpreter='latex');
xlim([0 limitx]);
legend()
saveas(gcf, 'figures/openLoop_velocities.png')


figure(Name= "Pitch angle, frequency and dihedral angle");
t = tiledlayout(3, 1);
title(t, 'Open Loop $\theta = 15 \deg$, various', Interpreter='Latex');

nexttile();
plot(sim_theta_t, sim_theta, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, onboard.thetaF, DisplayName= 'Data')
ylabel('$\theta \; [\deg/s]$', Interpreter='latex');
xlabel('$Time [s]$', Interpreter='latex');
xlim([0 limitx]);

nexttile();
plot(time, onboard.ff/60);
ylabel("$f \; [Hz]$", Interpreter="latex");
xlim([0 limitx]);

nexttile()
plot(sim_dih_t, sim_dih, DisplayName= 'model', Linestyle= '--'); hold on;
ylabel("$\Gamma [rad]$", Interpreter= 'latex')
xlim([0 limitx]);
xlabel("$Time [s]$", Interpreter= "latex");
legend();
saveas(gcf, 'figures/openLoop_various.png')
