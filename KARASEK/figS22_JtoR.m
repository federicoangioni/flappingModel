close all
clear all

addpath('fly_data')
addpath('support_files')

import_and_sort_fly_data

%% fly figures
colors=viridis(170); % 30 to 200
color1=colors(round(200-PITCH2ROLL1*100),:);
color2=colors(round(200-PITCH2ROLL2*100),:);
color3=colors(round(200-PITCH2ROLL3*100),:);
color4=colors(round(200-PITCH2ROLL4*100),:);


TIME_plot=TIME-TIME(1,1);

figure
colormap(colors);
caxis([0.3 2.1])
c=colorbar;
c.Label.String = 'q/p ratio';

figure
subplot(3,3,1)
% plot(TIME,OMX/pi*180/f_fly,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(OMX(:,N45)/pi*180/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(OMX(:,N90)/pi*180/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(OMX(:,N135)/pi*180/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(OMX(:,N180)/pi*180/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on
% plot(t_wb_seq_mean_all+time0,roll_dot_mean_wb_seq_mean_all_c1/f_fly,'bx')
% plot(t_wb_seq_mean_all+time0,roll_dot_mean_wb_seq_mean_all_c2/f_fly,'cx')
% plot(t_wb_seq_mean_all+time0,roll_dot_mean_wb_seq_mean_all_c3/f_fly,'kx')
% plot(t_wb_seq_mean_all+time0,roll_dot_mean_wb_seq_mean_all_c4/f_fly,'rx')
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('p (wingbeat^-^1)')
xlim([0 15])
ylim([-12 12])
box off

subplot(3,3,4)
% plot(TIME,OMY/pi*180/f_fly,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(OMY(:,N45)/pi*180/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(OMY(:,N90)/pi*180/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(OMY(:,N135)/pi*180/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(OMY(:,N180)/pi*180/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on
% plot(t_wb_seq_mean_all+time0,pitch_dot_mean_wb_seq_mean_all_c1/f_fly,'bx')
% plot(t_wb_seq_mean_all+time0,pitch_dot_mean_wb_seq_mean_all_c2/f_fly,'cx')
% plot(t_wb_seq_mean_all+time0,pitch_dot_mean_wb_seq_mean_all_c3/f_fly,'kx')
% plot(t_wb_seq_mean_all+time0,pitch_dot_mean_wb_seq_mean_all_c4/f_fly,'rx')
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('q (wingbeat^-^1)')
xlim([0 15])
ylim([-12 12])
box off

subplot(3,3,7)
% plot(TIME,OMZ/pi*180/f_fly,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(OMZ(:,N45)/pi*180/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(OMZ(:,N90)/pi*180/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(OMZ(:,N135)/pi*180/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(OMZ(:,N180)/pi*180/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on
% plot(t_wb_seq_mean_all+time0,yaw_dot_mean_wb_seq_mean_all_c1/f_fly,'bx')
% plot(t_wb_seq_mean_all+time0,yaw_dot_mean_wb_seq_mean_all_c2/f_fly,'cx')
% plot(t_wb_seq_mean_all+time0,yaw_dot_mean_wb_seq_mean_all_c3/f_fly,'kx')
% plot(t_wb_seq_mean_all+time0,yaw_dot_mean_wb_seq_mean_all_c4/f_fly,'rx')
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('r (wingbeat^-^1)')
xlim([0 15])
ylim([-12 12])
box off
xlabel('wingbeats')

% figure
subplot(3,3,2)
% plot(TIME_plot,VELX,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(VELX(:,N45)/R_fly/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(VELX(:,N90)/R_fly/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(VELX(:,N135)/R_fly/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(VELX(:,N180)/R_fly/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on

% ylabel('u (m/s)')
ylabel('u (R/wingb.)')
ylim([-0.1 0.5])
xlim([0 15])
box off

subplot(3,3,5)
% plot(TIME_plot,VELY,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(VELY(:,N45)/R_fly/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(VELY(:,N90)/R_fly/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(VELY(:,N135)/R_fly/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(VELY(:,N180)/R_fly/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on

% ylabel('v (m/s)')
ylabel('v (R/wingb.)')
ylim([-0.1 0.5])
xlim([0 15])
box off

subplot(3,3,8)
% plot(TIME_plot,VELZ,'Color',[.8 .8 .8]),hold on
plot(TIME_plot(:,N45)*f_fly,mean(VELZ(:,N45)/R_fly/f_fly,2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(VELZ(:,N90)/R_fly/f_fly,2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(VELZ(:,N135)/R_fly/f_fly,2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(VELZ(:,N180)/R_fly/f_fly,2,'omitnan'),'Color',color4,'LineWidth',2),hold on

% ylabel('w (m/s)')
ylabel('w (R/wingb.)')
ylim([-0.5 0.1])
xlim([0 15])
box off

% figure
subplot(3,3,3)
plot(TIME_plot(:,N45)*f_fly,mean(BET(:,N45),2,'omitnan')-mean(BET(1,N45),2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(BET(:,N90),2,'omitnan')-mean(BET(1,N90),2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(BET(:,N135),2,'omitnan')-mean(BET(1,N135),2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(BET(:,N180),2,'omitnan')-mean(BET(1,N180),2,'omitnan'),'Color',color4,'LineWidth',2),hold on
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('\Delta sideslip')
xlim([0 15])
ylim([-20 100])
box off

subplot(3,3,6)
plot(TIME_plot(:,N45)*f_fly,mean(COURSE(:,N45),2,'omitnan')/pi*180,'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(COURSE(:,N90),2,'omitnan')/pi*180,'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(COURSE(:,N135),2,'omitnan')/pi*180,'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(COURSE(:,N180),2,'omitnan')/pi*180,'Color',color4,'LineWidth',2),hold on
xlim([0 15])
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('\Delta course')
ylim([-20 180])
box off

subplot(3,3,9)
plot(TIME_plot(:,N45)*f_fly,mean(VELOC(:,N45),2,'omitnan')-mean(VELOC(1,N45),2,'omitnan'),'Color',color1,'LineWidth',2),hold on
plot(TIME_plot(:,N90)*f_fly,mean(VELOC(:,N90),2,'omitnan')-mean(VELOC(1,N90),2,'omitnan'),'Color',color2,'LineWidth',2),hold on
plot(TIME_plot(:,N135)*f_fly,mean(VELOC(:,N135),2,'omitnan')-mean(VELOC(1,N135),2,'omitnan'),'Color',color3,'LineWidth',2),hold on
plot(TIME_plot(:,N180)*f_fly,mean(VELOC(:,N180),2,'omitnan')-mean(VELOC(1,N180),2,'omitnan'),'Color',color4,'LineWidth',2),hold on
xlim([0 15])
ylim([-0.25 0.35])
box off

ylabel('\Delta U (R/wingb.)')
xlabel('wingbeats')

