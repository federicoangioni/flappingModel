close all
clear all

addpath('support_files')
load dataset_revision.mat

Nexp=2;
    
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
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[PITCHintavg'+nSigma*PITCHintstd';flipud(PITCHintavg'-nSigma*PITCHintstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,PITCHint,'b-')
plot(TIME,PITCHintavg,'k-','LineWidth',2)
% grid on
ylabel('integrated pitch (deg)')
set(gca,'Ylim',[-360 20])
set(gca,'YTick',[-360:90:0])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(5)=subplot(3,3,5);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[OMyavg'+nSigma*OMystd';flipud(OMyavg'-nSigma*OMystd')],[.9 .9 .9],'linestyle','none');
plot(TIME,OMy,'b-')
plot(TIME,OMyavg,'k-','LineWidth',2)
% grid on
ylabel('q (deg/s)')
set(gca,'Ylim',[-1200, 600])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(8)=subplot(3,3,8);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[ALPHyavg'+nSigma*ALPHystd';flipud(ALPHyavg'-nSigma*ALPHystd')],[.9 .9 .9],'linestyle','none');
plot(TIME,ALPHy,'b-')
plot(TIME,ALPHyavg,'k-','LineWidth',2)
% grid on
ylabel('dq/dt (deg/s^2)')
xlabel('time (s)')
set(gca,'Ylim',[-7500 7500])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(3)=subplot(3,3,3);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[POSzavg'+nSigma*POSzstd';flipud(POSzavg'-nSigma*POSzstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,-POSz,'b-'),hold on
plot(TIME,-POSzavg,'k-','LineWidth',2)
% grid on
ylabel('z (m)')
set(gca,'Ylim',[-1 1])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(6)=subplot(3,3,6);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[VELEzavg'+nSigma*VELEzstd';flipud(VELEzavg'-nSigma*VELEzstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,-VELEz,'b-'),hold on
plot(TIME,-VELEzavg,'k-','LineWidth',2)
% grid on
ylabel('v_z (m/s)')
set(gca,'Ylim',[-2.5 2])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(9)=subplot(3,3,9);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],-[ACCEzavg'+nSigma*ACCEzstd';flipud(ACCEzavg'-nSigma*ACCEzstd')]/g,[.9 .9 .9],'linestyle','none');
plot(TIME,-ACCEz/g,'b-'),hold on
plot(TIME,-ACCEzavg/g,'k-','LineWidth',2)
% grid on
ylabel('a_z (g)')
xlabel('time (s)')
set(gca,'Ylim',[-2.6 1.6])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(1)=subplot(3,3,1);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[CMDpitchavg'+nSigma*CMDpitchstd';flipud(CMDpitchavg'-nSigma*CMDpitchstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,CMDpitch,'b-'),hold on
plot(TIME,CMDpitchavg,'k-','LineWidth',2)
ylabel('Pitch command (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[-100 100])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,1)

ax_flip_angular(4)=subplot(3,3,4);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[CMDrightavg'+nSigma*CMDrightstd';flipud(CMDrightavg'-nSigma*CMDrightstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,CMDright,'b-'),hold on
plot(TIME,CMDrightavg,'k-','LineWidth',2)
ylabel('Right wing command (%)')
% set(gca,'XLim',[start_times(1)-2,start_times(1)+2])
set(gca,'Ylim',[20 100])
set(gca,'XTickLabels',[])
wingbeat_axis_top(gca,Nwingbeats,0)

ax_flip_angular(7)=subplot(3,3,7);
pitch_flip_plot_rectangles
fill([TIME';flipud(TIME')],[FREQavg'+nSigma*FREQstd';flipud(FREQavg'-nSigma*FREQstd')],[.9 .9 .9],'linestyle','none');
plot(TIME,FREQ,'b-'),hold on
plot(TIME,FREQavg,'k-','LineWidth',2)
ylabel('Right w. f (Hz)')
xlabel('time (s)')
set(gca,'Ylim',[5 25])
wingbeat_axis_top(gca,Nwingbeats,0)

linkaxes([ax_flip_angular],'x')
set(ax_flip_angular(1),'Xlim',[-1,1.5])
