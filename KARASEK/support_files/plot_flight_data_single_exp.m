figure('Position',[100 100 1150 470])

ax_flip_angular(1)=subplot(4,5,1);
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_roll,'b')
ylabel('roll cmd (%)')
set(gca,'Ylim',[-100 100])
set(gca,'XColor','none');
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(6)=subplot(4,5,6);
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_pitch,'b')
ylabel('pitch cmd (%)')
set(gca,'Ylim',[-100 100])
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(11)=subplot(4,5,11);
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_yaw,'b'),hold on
ylabel('yaw cmd (%)')
set(gca,'Ylim',[-100 100])
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(16)=subplot(4,5,16);
evasive_turn_plot_rectangles2
plot(time_onboard,cmd_thrust,'b'),hold on
ylabel('throttle cmd (%)')
set(gca,'Ylim',[0 100])
% wingbeat_axis_top(gca,Nwingbeats,0)
xlabel('time (s)')

ax_flip_angular(2)=subplot(4,5,2);
evasive_turn_plot_rectangles2
plot(time,roll,'b-')
% grid on
ylabel('roll')
set(gca,'Ylim',[-20 110])
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(7)=subplot(4,5,7);
evasive_turn_plot_rectangles2
plot(time,pitch,'b-')
% grid on
ylabel('pitch')
set(gca,'Ylim',[-60 40])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(12)=subplot(4,5,12);
evasive_turn_plot_rectangles2
plot(time,yaw_aligned,'b-')
% grid on
ylabel('yaw')
set(gca,'Ylim',[-20 100])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(17)=subplot(4,5,17);
evasive_turn_plot_rectangles2
plot(time,head_aligned,'b-')
% grid on
ylabel('course')
set(gca,'Ylim',[-20 150])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
% wingbeat_axis_top(gca,Nwingbeats,0)
xlabel('time (s)')

ax_flip_angular(3)=subplot(4,5,3);
evasive_turn_plot_rectangles2
plot(time,omx,'b-')
% grid on
ylabel('roll rate (s^-^1)')
set(gca,'Ylim',[-600 600])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(8)=subplot(4,5,8);
evasive_turn_plot_rectangles2
plot(time,omy,'b-')
% grid on
ylabel('pitch rate (s^-^1)')
set(gca,'Ylim',[-600 600])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(13)=subplot(4,5,13);
evasive_turn_plot_rectangles2
plot(time,omz,'b-')
% grid on
ylabel('yaw rate (s^-^1)')
set(gca,'Ylim',[-600 600])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(18)=subplot(4,5,18);
evasive_turn_plot_rectangles2
plot(time,dhead,'b-')
% grid on
ylabel('turn rate (s^-^1)')
set(gca,'Ylim',[-200 800])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
% wingbeat_axis_top(gca,Nwingbeats,0)
xlabel('time (s)')

ax_flip_angular(4)=subplot(4,5,4);
evasive_turn_plot_rectangles2
plot(time,alphx,'b-')
ylabel('roll acc. (s^-^2)')
set(gca,'Ylim',[-6000 6000])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(9)=subplot(4,5,9);
evasive_turn_plot_rectangles2
plot(time,alphy,'b-')
ylabel('pitch acc. (s^-^2)')
set(gca,'Ylim',[-6000 6000])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(14)=subplot(4,5,14);
evasive_turn_plot_rectangles2
plot(time,alphz,'b-')
ylabel('yaw acc. (s^-^2)')
set(gca,'Ylim',[-6000 6000])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';
set(gca,'XColor','none'); 
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(19)=subplot(4,5,19);
evasive_turn_plot_rectangles2
plot(time,veloc,'b-')
ylabel('speed (m.s^-^1)')
set(gca,'Ylim',[-0.5 3.5])
% wingbeat_axis_top(gca,Nwingbeats,0)
xlabel('time (s)')


ax_flip_angular(5)=subplot(4,5,5);
evasive_turn_plot_rectangles2
plot(time_freq,freq_right,'b')
ylabel('right w. freq. (Hz)')
set(gca,'Ylim',[10 25])
set(gca,'XColor','none');
% wingbeat_axis_top(gca,Nwingbeats,0)


ax_flip_angular(10)=subplot(4,5,10);
evasive_turn_plot_rectangles2
plot(time,dihed,'b'),hold on
ylabel('dihedral')
set(gca,'Ylim',[-25 25])
set(gca,'XColor','none');
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';

ax_flip_angular(20)=subplot(4,5,15);
evasive_turn_plot_rectangles2
plot(time,bet,'b-')
ylabel('sideslip')
xlabel('time (s)')
set(gca,'Ylim',[-20 90])
ca=gca; ca.YAxis.TickLabelFormat = '%g\\circ';

linkaxes([ax_flip_angular],'x')
set(ax_flip_angular(1),'Xlim',[-0.2,1.2])

for i=1:5
    subplot(4,5,i);
    wingbeat_axis_top(gca,Nwingbeats,1)
end
