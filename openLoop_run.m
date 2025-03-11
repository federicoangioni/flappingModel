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
simOut = sim( 'OL_fullnonlin_prevval_ucorr.slx', 'ExternalInput', cell_input{1}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);
cd('..')

yout = get(simOut,'yout');

% simout

sim_ud = simOut.yout{1}.Values.Data;
sim_ud_time = simOut.yout{1}.Values.Time;

sim_thetadd = simOut.yout{5}.Values.Data;
sim_thetadd_t = simOut.yout{5}.Values.Time;

sim_wd = simOut.yout{3}.Values.Data;
sim_wd_time = simOut.yout{3}.Values.Time;

t = tiledlayout(4, 1);

nexttile
title('OL with new data minimal longitudinal')
plot(sim_ud_time, sim_ud, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.udFF, DisplayName= 'data')
ylabel('$\dot{u} \quad [m/s]$', Interpreter='latex')
xlim([0 2.5])
ylim([-5 5])
legend()

nexttile

plot(sim_wd_time, sim_wd, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.wdFF, DisplayName= 'data')
ylabel('$\dot{w} \quad [m/s]$', Interpreter='latex')
xlim([0 2.5])
ylim([-5 5])
nexttile
plot(sim_thetadd_t, sim_thetadd, DisplayName= 'model', LineStyle='--'); hold on;
plot(time, optitrack.thetaddFF, DisplayName='Data')
ylabel('$\ddot{\theta} \quad [rad/s]$', Interpreter='latex')
ylim([-20 20])
xlim([0 2.5])
nexttile

plot(input_data{1}(:, 1), pprz_filt)
ylabel('$ PPRZ \: cmd \: [\%] $', Interpreter='latex')

title(t, 'OL configuration Pitch Maneuver 15 deg')
saveas(gcf, 'figures/OL_model_newdata.png')

