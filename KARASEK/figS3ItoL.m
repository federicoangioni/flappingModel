close all
clear all
clc

addpath('force_balance_data')

load force_balance_yaw_steps.mat

%% detecting steps
index=1:length(time);
samplesHIGH=zeros(size(time));
samplesHIGH(CMDyaw>mean(CMDyaw))=1;
indexUP=index([diff(samplesHIGH),0]==1);
timesUP=time(indexUP);
indexDOWN=index([diff(samplesHIGH),0]==-1);
timesDOWN=time(indexDOWN);

%% science figures
limMIN=-7;
limMAX=7;

t0=timesUP(2);
t1=timesDOWN(2);

figure('pos',[10 110 250 400])
ax(1)=subplot(4,1,1);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,-100,100)
plot(time-t0,CMDyaw,'k')
ylabel('cmd_y_a_w (%)')
set(gca,'Ylim',[-100,100])


ax(2)=subplot(4,1,2);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)

plot(time-t0,MxF), hold on
% legend('filtered','moving average')
ylabel('L (Nmm)')
set(gca,'Ylim',[limMIN,limMAX])

ax(3)=subplot(4,1,3);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)
plot(time-t0,MyF), hold on
% legend('filtered','moving average')
ylabel('M (Nmm)')
set(gca,'Ylim',[limMIN,limMAX])

ax(4)=subplot(4,1,4);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)
plot(time-t0,MzF)
% legend('filtered','moving average')
ylabel('N (Nmm)')
xlabel('time (s)')
% legend('Moment filtered @ 5Hz','scaled PWM command')
set(gca,'Ylim',[-3,3])

linkaxes(ax,'x')
set(gca,'Xlim',[-0.1,0.95])
set(gca,'Xtick',[0:0.2:1])

