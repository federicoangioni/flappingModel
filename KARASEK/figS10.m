close all
clear all

Nexp=4;
nman=2;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables

g=9.81;

%% filter angles
fps=1/median(diff(time));
Fcut=10;

[b,a] = butter(8,2*Fcut/fps); % the argument is normalized frequency in pi*rad/sample, thus it needs to be multiplied by 2 (2*pi rad = 1 cycle)

rollF=LP_filter(b,a,roll);
pitchF=LP_filter(b,a,pitch);

%% Supplement - roll step new
Nwingbeats=4*17;

tt0=(70535+14*36-106)/fps;
tt1=(70535+14*36)/fps;
tt2=(70535+14*36+21*36)/fps;
tt3=(70535+14*36-106+468)/fps;

figure
ax_flip_angular(2)=subplot(3,3,2);
pitch_step_plot_rectangles
plot(time,rollF,'b-')
plot(time_onboard,roll_onboard,'b:')
plot(time_onboard,rc_roll,'b--')
ylabel('roll (deg)')
set(gca,'Ylim',[-70 30])
set(gca,'XTickLabels',[])
legend('OptiTrack','IMU','reference')
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(5)=subplot(3,3,5);
pitch_step_plot_rectangles
plot(time,omx,'b-')
plot(time_onboard_rates, roll_rate_onboard,'b:')
ylabel('p (deg/s)')
set(gca,'Ylim',[-400 500])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)


ax_flip_angular(8)=subplot(3,3,8);
pitch_step_plot_rectangles
plot(time,alphx,'b-')
plot(time_onboard_rates, roll_acc_onboard,'b:')
ylabel('dp/dt (deg/s^2)')
xlabel('time (s)')
set(gca,'Ylim',[-6000 6000])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(3)=subplot(3,3,3);
pitch_step_plot_rectangles
plot(time,posY_aligned,'b-'),hold on
ylabel('y (m)')
set(gca,'Ylim',[-4.5 0.5])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(6)=subplot(3,3,6);
pitch_step_plot_rectangles
plot(time,vel_CG_E_aligned(2,:),'b-'),hold on
ylabel('v_y (m/s)')
set(gca,'Ylim',[-4.5 0.5])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(9)=subplot(3,3,9);
pitch_step_plot_rectangles
plot(time,acc_CG_E_aligned_y/g,'b-'),hold on
ylabel('a_y (g)')
xlabel('time (s)')
set(gca,'Ylim',[-1 2])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(1)=subplot(3,3,1);
pitch_step_plot_rectangles
plot(time_onboard,cmd_roll,'b'),hold on
ylabel('Roll cmd (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[-100 100])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(4)=subplot(3,3,4);
pitch_step_plot_rectangles
plot(time_freq,freq_right,'b'),hold on
ylabel('Right wing freq. (Hz)')
set(gca,'Ylim',[14 24])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(7)=subplot(3,3,7);
pitch_step_plot_rectangles
% plot(time_freq,motor_rpm/21.33,'b')
plot(time_onboard,cmd_thrust,'b')
% plot(time_onboard,cmd_motor_rightFF/100*25,'b:')
% plot(time_onboard,cmd_motor_leftFF/100*25,'b:')
ylabel('Thrust cmd (%)')
xlabel('time (s)')
set(gca,'Ylim',[0 100])
wingbeat_axis_top(gca,Nwingbeats,0)

linkaxes([ax_flip_angular],'x')
set(ax_flip_angular(1),'Xlim',[-1,3])