close all
clear all

Nexp=4;
nman=1;

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

%% Supplement - pitch step new
Nwingbeats=4*17;

tt0=(45700+14*36-106)/fps;
tt1=(45700+14*36)/fps;
tt2=(45700+14*36+21*36)/fps;
tt3=(45700+14*36-106+407)/fps;

figure
ax_flip_angular(2)=subplot(3,3,2);
pitch_step_plot_rectangles
plot(time,pitchF,'b-')
plot(time_onboard,pitch_onboard,'b:')
plot(time_onboard,rc_pitch,'b--')
% plot(time,PITCHavg,'k-','LineWidth',2)
% grid on
ylabel('pitch (deg)')
set(gca,'Ylim',[-80 20])
set(gca,'XTickLabels',[])
legend('OptiTrack','IMU','reference')
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(5)=subplot(3,3,5);
pitch_step_plot_rectangles
% fill([time';flipud(time')],[OMyavg'+nSigma*OMystd';flipud(OMyavg'-nSigma*OMystd')],[.9 .9 .9],'linestyle','none');
plot(time,omy,'b-')
plot(time_onboard_rates, pitch_rate_onboard,'b:')
% plot(time,OMyavg,'k-','LineWidth',2)
% grid on
ylabel('q (deg/s)')
set(gca,'Ylim',[-400 400])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)


ax_flip_angular(8)=subplot(3,3,8);
pitch_step_plot_rectangles
% fill([time';flipud(time')],[ALPHyavg'+nSigma*ALPHystd';flipud(ALPHyavg'-nSigma*ALPHystd')],[.9 .9 .9],'linestyle','none');
plot(time,alphy,'b-')
plot(time_onboard_rates, pitch_acc_onboard,'b:')
% plot(time,ALPHyavg,'k-','LineWidth',2)
% grid on
ylabel('dq/dt (deg/s^2)')
xlabel('time (s)')
set(gca,'Ylim',[-4000 4000])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(3)=subplot(3,3,3);
pitch_step_plot_rectangles
% fill([time';flipud(time')],-[POSzavg'+nSigma*POSzstd';flipud(POSzavg'-nSigma*POSzstd')],[.9 .9 .9],'linestyle','none');
plot(time,posX_aligned,'b-'),hold on
% plot(time,-POSzavg,'k-','LineWidth',2)
% grid on
ylabel('x (m)')
set(gca,'Ylim',[-0.5 4.5])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(6)=subplot(3,3,6);
pitch_step_plot_rectangles
% fill([time';flipud(time')],-[VELEzavg'+nSigma*VELEzstd';flipud(VELEzavg'-nSigma*VELEzstd')],[.9 .9 .9],'linestyle','none');
plot(time,vel_CG_E_aligned(1,:),'b-'),hold on
% plot(time,-VELEzavg,'k-','LineWidth',2)
% grid on
ylabel('v_x (m/s)')
set(gca,'Ylim',[-0.5 4.5])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(9)=subplot(3,3,9);
pitch_step_plot_rectangles
% fill([time';flipud(time')],-[ACCEzavg'+nSigma*ACCEzstd';flipud(ACCEzavg'-nSigma*ACCEzstd')]/g,[.9 .9 .9],'linestyle','none');
plot(time,acc_CG_E_aligned_x/g,'b-'),hold on
% plot(time,-ACCEzavg/g,'k-','LineWidth',2)
% grid on
ylabel('a_x (g)')
xlabel('time (s)')
set(gca,'Ylim',[-2 1])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(1)=subplot(3,3,1);
pitch_step_plot_rectangles
plot(time_onboard,cmd_pitch,'b'),hold on
ylabel('Pitch ser. cmd (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[-100 100])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(4)=subplot(3,3,4);
pitch_step_plot_rectangles
plot(time,dihed,'b')
ylabel('Dihedral (deg)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[-30 30])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(7)=subplot(3,3,7);
pitch_step_plot_rectangles
% plot(time_rpm_log-time0-start_times(1),motor_rpm/21.33,'b')
plot(time_onboard,cmd_thrust,'b')
% plot(time_onboard,cmd_motor_rightFF/100*25,'b:')
% plot(time_onboard,cmd_motor_leftFF/100*25,'b:')
ylabel('Thrust cmd (%)')
xlabel('time (s)')
set(gca,'Ylim',[0 100])
wingbeat_axis_top(gca,Nwingbeats,0)

linkaxes([ax_flip_angular],'x')
set(ax_flip_angular(1),'Xlim',[-1,3])
