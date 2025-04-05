close all
clear all

Nexp=13;
nman=1; % I only saved one maneuver in the Science dataset, it is number 2 in the original data

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables
fps=1/median(diff(time));

% filtering actuators (same filter as used for the accelerations)
FcutStrong=10; % used for rates, velocities and accelerations

fs_freq=1/median(diff(time_freq));
[b,a] = butter(8,2*FcutStrong/fs_freq); % the argument is normalized frequency in pi*rad/sample, thus it needs to be multiplied by 2 (2*pi rad = 1 cycle)
freq_right_filt=LP_filter(b,a,freq_right);

fs_dihed=1/median(diff(time));
[b,a] = butter(8,2*FcutStrong/fs_dihed); % the argument is normalized frequency in pi*rad/sample, thus it needs to be multiplied by 2 (2*pi rad = 1 cycle)
dihed_filt=LP_filter(b,a,dihed);
dihed_filt(isnan(dihed))=NaN;


Nwingbeats=(9/fps+1.25)*17;

figure

ax_flip(1)=subplot(4,1,1);
% yyaxis left
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_roll),hold on
ylabel('Roll cmd (%)')
set(gca,'Ylim',[-100 100])

set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip(2)=subplot(4,1,3);

% yyaxis left
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_pitch)
ylabel('Pitch cmd (%)')
set(gca,'Ylim',[-100 100])

ax_flip(3)=subplot(4,1,2);
yyaxis left
evasive_turn_plot_rectangles2
plot(time, alphx),hold on 
ylabel('dp/dt (deg/s^2)')
set(gca,'Ylim',[-6000 6000])
set(gca,'Ydir','reverse')
set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,0)

yyaxis right
plot(time_freq,freq_right_filt)
ylabel('Right w. f (Hz)')
set(gca,'Ylim',[10 25])
set(gca,'XTickLabels',[])

ax_flip(4)=subplot(4,1,4);
yyaxis left
evasive_turn_plot_rectangles2
plot(time, alphy),hold on 
ylabel('dq/dt (deg/s^2)')
set(gca,'Ylim',[-4000 4000])
xlabel('time (s)')
% wingbeat_axis_top(gca,Nwingbeats,0)

yyaxis right
plot(time, dihed_filt),hold on
ylabel('Dihedral (deg)')
set(gca,'Ylim',[-25 5])


linkaxes([ax_flip],'x')
set(ax_flip(1),'Xlim',[-9/fps,-9/fps+1.25])
