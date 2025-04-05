close all
clear all

Nexp=56;
nman=3;

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

Nwingbeats=3.25*17;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables
plot_flight_data_single_exp