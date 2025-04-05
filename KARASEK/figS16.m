close all
clear all

Nexp=36;
nman=1; % I only saved one maneuver in the dataset, it is number 5 in the original data

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

Nwingbeats=3.25*17;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables
plot_flight_data_single_exp