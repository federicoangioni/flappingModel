close all
clear all

addpath('support_files')
load dataset_revision.mat

Nexp=1;
    
eval(['data=experiment' num2str(Nexp) ';'])
assign_repetition_variables

g=9.81;
    
%% Supplement - roll flip

nSigma=3; % number of sigmas to show in error bands

Nwingbeats=2.5*17;
% ROLLavgold=ROLLavg;
% make roll continuous
% for i=2:length(ROLLavg)
%     if(ROLLavg(i)-ROLLavg(i-1))<-10
%         ROLLavg(i:end)=ROLLavg(i:end)+360;
%         break
%     end
% end

% integration of body rates
fps=1/median(diff(TIME));
ROLLint=zeros(size(ROLL));
PITCHint=zeros(size(PITCH));
YAWint=zeros(size(YAW));

for i=2:length(TIME)
    ROLLint(:,i)=ROLLint(:,i-1)+OMx(:,i)*1/fps;
    PITCHint(:,i)=PITCHint(:,i-1)+OMy(:,i)*1/fps;
    YAWint(:,i)=YAWint(:,i-1)+OMz(:,i)*1/fps;
end

% set zero reference to the start of the maneuver
ROLLint=ROLLint-repmat(ROLLint(:,TIME==0),1,length(TIME));
PITCHint=PITCHint-repmat(PITCHint(:,TIME==0),1,length(TIME));
YAWint=YAWint-repmat(YAWint(:,TIME==0),1,length(TIME));

ROLLintavg=mean(ROLLint);
ROLLintstd=std(ROLLint);
PITCHintavg=mean(PITCHint);
PITCHintstd=std(PITCHint);
YAWintavg=mean(YAWint);
YAWintstd=std(YAWint);
        

figure
ax_flip_angular(2)=subplot(3,3,2);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],[ROLLintavg'+nSigma*ROLLintstd';flipud(ROLLintavg'-nSigma*ROLLintstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,ROLLint,'b-')
plot(TIME,ROLLintavg,'k-','LineWidth',2)
% grid on
ylabel('integrated roll rate (deg)')
set(gca,'Ylim',[-30 390])
set(gca,'YTick',[0:90:360])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(5)=subplot(3,3,5);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],[OMxavg'+nSigma*OMxstd';flipud(OMxavg'-nSigma*OMxstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,OMx,'b-')
plot(TIME,OMxavg,'k-','LineWidth',2)
% grid on
ylabel('p (deg/s)')
set(gca,'Ylim',[-600 1200])
set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(8)=subplot(3,3,8);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],[ALPHxavg'+nSigma*ALPHxstd';flipud(ALPHxavg'-nSigma*ALPHxstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,ALPHx,'b-')
plot(TIME,ALPHxavg,'k-','LineWidth',2)
% grid on
ylabel('dp/dt (deg/s^2)')
xlabel('time (s)')
set(gca,'Ylim',[-7500 7500])
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(3)=subplot(3,3,3);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[POSzavg'+nSigma*POSzstd';flipud(POSzavg'-nSigma*POSzstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,-POSz,'b-'),hold on
plot(TIME,-POSzavg,'k-','LineWidth',2)
% grid on
ylabel('z (m)')
set(gca,'Ylim',[-1 1])
set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(6)=subplot(3,3,6);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[VELEzavg'+nSigma*VELEzstd';flipud(VELEzavg'-nSigma*VELEzstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,-VELEz,'b-'),hold on
plot(TIME,-VELEzavg,'k-','LineWidth',2)
% grid on
ylabel('v_z (m/s)')
set(gca,'Ylim',[-2.5 2])
set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(9)=subplot(3,3,9);
roll_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[ACCEzavg'+nSigma*ACCEzstd';flipud(ACCEzavg'-nSigma*ACCEzstd')]/g,[.9 .9 .9],'linestyle','none');
plot(TIME,-ACCEz/g,'b-'),hold on
plot(TIME,-ACCEzavg/g,'k-','LineWidth',2)
% grid on
ylabel('a_z (g)')
xlabel('time (s)')
set(gca,'Ylim',[-2.6 1.6])
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(1)=subplot(3,3,1);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[CMDleftavg'+nSigma*CMDleftstd';flipud(CMDleftavg'-nSigma*CMDleftstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,CMDleft,'b-'),hold on
plot(TIME,CMDleftavg,'k-','LineWidth',2)
ylabel('Left wing command (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[20 100])
set(gca,'XTickLabels',[])

ax_flip_angular(4)=subplot(3,3,4);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[CMDrightavg'+nSigma*CMDrightstd';flipud(CMDrightavg'-nSigma*CMDrightstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,CMDright,'b-'),hold on
plot(TIME,CMDrightavg,'k-','LineWidth',2)
ylabel('Right wing command (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[20 100])
set(gca,'XTickLabels',[])
% wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(7)=subplot(3,3,7);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[FREQavg'+nSigma*FREQstd';flipud(FREQavg'-nSigma*FREQstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,FREQ,'b-'),hold on
plot(TIME,FREQavg,'k-','LineWidth',2)
ylabel('Right w. f (Hz)')
xlabel('time (s)')
set(gca,'Ylim',[5 25])
% wingbeat_axis_top(gca,Nwingbeats,0)

linkaxes([ax_flip_angular],'x')
set(ax_flip_angular(1),'Xlim',[-1,1.5])