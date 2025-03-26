clc; clearvars;
%% Import the data

load data\BAL15_set.mat;

% Now it is important to input the correct data
% Opti has both velocities and angles
% onboard has only angles and commands

optitrack = testsids{1}.opti;
onboard = testsids{1}.onboard;


pprz_pitch = onboard.pitchcmd'*96;

time = testsids{1}.t';

%% Specify inputs to model
fs = 360; %Hz
fc = 5; % Hz

order = 4;

[b, a] = butter(order, fc/(fs/2), 'low'); 

pprz_filt = filtfilt(b, a, pprz_pitch);

%% Inputs
% prepare input data to be processed by the model
input_data = Simulink.SimulationData.Dataset();
input_data = input_data.addElement([time pprz_filt], 'PPRZ_CMD');

%% Specify model parameters

para.m = 0.0294;

para.Iyy = 1.26e-4;

para.bx = 0.0722;

para.bz = 0.0157;

para.lw = 0.081;

para.lz = 0.0271;

para.omega = 40;

para.chi = 0.634;

para.ucorr = 10;

para.c1 = 0.0114;

para.c2 = -0.0449;

para.f = 16.584013596491230;

para.w0 = 0.1217;

para.I = 1.26e-4;
% watch out different lz initial conditions lead to very different results
%% Set start time

earliestsetpointstarttime = 9999999;
earliestendtime = 9999999;


findpts = testsids{1, 1}.onboard.thetacmd/pi*180;
findpts(abs(findpts)/pi*180<10) = 0;
earliestind = find(findpts,1);
earliesttime = testsids{1, 1}.t(earliestind);

testsids{1, 1}.setpointstarttime = earliesttime;

cell_input = {'input_data.getElement(1)'};

dataarray = input_data.get(1);
stoptime = dataarray(end,1);

simOut = sim("OL_model.slx", 'ExternalInput', cell_input{1}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);

yout = get(simOut,'yout');


%% Collect data from simulation

% Accelerations:
% u dot
sim_ud = simOut.yout{5}.Values.Data;
sim_ud_t = simOut.yout{5}.Values.Time;

% w dot
sim_wd = simOut.yout{6}.Values.Data;
sim_wd_t = simOut.yout{6}.Values.Time;

% theta double dot
sim_thetadd = simOut.yout{7}.Values.Data;
sim_thetadd_t = simOut.yout{7}.Values.Time;

% Velocities:
% u
sim_u = simOut.yout{1}.Values.Data;
sim_u_t = simOut.yout{1}.Values.Time;

% w
sim_w = simOut.yout{2}.Values.Data;
sim_w_t = simOut.yout{2}.Values.Time;

% thetad
sim_thetad = simOut.yout{4}.Values.Data;
sim_thetad_t = simOut.yout{4}.Values.Time;

% Pitch angle theta
sim_theta = simOut.yout{3}.Values.Data;
sim_theta_t = simOut.yout{3}.Values.Time;


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
plot(input_data{1}(:, 1), pprz_pitch);
ylabel('$ PPRZ \: cmd \: [\% \cdot 96] $', Interpreter='latex');
xlabel('$Time [s]$', Interpreter='latex');
xlim([0 limitx]);



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

