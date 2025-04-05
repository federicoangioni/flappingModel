clc; clearvars;
%% Import the data

cd('..');
dataset = 'experiment104';
data = load ('data\flight_data_torque_couplingSciencePaper.mat', dataset).(dataset);
cd('validate_dihedral');

onboard = data.onboard;


%% Summary of datasets

tiledlayout(3, 4);
i = 3;
nexttile();
plot(onboard.angles_commands_setpoints.TIME_onboard{i}, onboard.angles_commands_setpoints.PITCH_IMU{i})



