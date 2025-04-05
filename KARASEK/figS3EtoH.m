close all
clear all
clc

addpath('force_balance_data')

load force_balance_pitch_steps.mat

%% detecting steps
index=1:length(time);
samplesHIGH=zeros(size(time));
samplesHIGH(CMDpitch>mean(CMDpitch))=1;
indexUP=index([diff(samplesHIGH),0]==1);
timesUP=time(indexUP);
indexDOWN=index([diff(samplesHIGH),0]==-1);
timesDOWN=time(indexDOWN);

%% science figures

limMIN=-7;
limMAX=7;

t0=timesUP(1);
t1=timesDOWN(2);

figure('pos',[10 110 250 400])
ax(1)=subplot(4,1,1);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,-100,100)
plot(time-t0,CMDpitch,'k')
ylabel('cmd_p_i_t_c_h (%)')


ax(2)=subplot(4,1,2);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)

plot(time-t0,MxCOGF), hold on
% legend('filtered','moving average')
ylabel('L (Nmm)')
set(gca,'Ylim',[limMIN,limMAX])

ax(3)=subplot(4,1,3);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)
plot(time-t0,MyCOGF), hold on
% legend('filtered','moving average')
ylabel('M (Nmm)')
set(gca,'Ylim',[limMIN,limMAX])

ax(4)=subplot(4,1,4);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,limMIN,limMAX)
plot(time-t0,MzCOGF)
% legend('filtered','moving average')
ylabel('N (Nmm)')
xlabel('time (s)')
% legend('Moment filtered @ 5Hz','scaled PWM command')
set(gca,'Ylim',[limMIN,limMAX])

linkaxes(ax,'x')
set(gca,'Xlim',[-0.05,0.65])
set(gca,'Xtick',[0:0.2:0.6])

