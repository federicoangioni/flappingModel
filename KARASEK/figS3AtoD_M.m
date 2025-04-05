close all
clear all
clc

addpath('force_balance_data')

load force_balance_roll_steps.mat

%% detecting steps
index=1:length(time);
samplesHIGH=zeros(size(time));
samplesHIGH(CMDmotorLeft>mean(CMDmotorLeft))=1;
indexUP=index([diff(samplesHIGH),0]==1);
timesUP=time(indexUP);
indexDOWN=index([diff(samplesHIGH),0]==-1);
timesDOWN=time(indexDOWN);

%% science figures

limMIN=-7;
limMAX=7;

t0=timesUP(1);
t1=timesDOWN(1);

figure('pos',[10 110 250 400])
ax(1)=subplot(4,1,1);
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,-100,100)
% plot(time,(CMDmotorLeft-(max(CMDmotorLeft)+min(CMDmotorLeft))/2)/(1.832-1.148)*100,'k')
plot(time-t0,CMDmotorLeft-CMDmotorRight,'k')
ylabel('cmd_r_o_l_l (%)')
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
set(gca,'Ylim',[limMIN,limMAX])

linkaxes(ax,'x')
set(gca,'Xlim',[-0.1,1.1])
set(gca,'Xtick',[0:0.2:1])

%%
% actuator dynamics (identified by Karl)
    
% f = s1*cmd + s2, values hand tuned
s1L = 0.2014*1.33;
s2L = 3.9517*0.52; % 90% correction to fit the data

s1R = 0.2014*1.38;
s2R = 3.9517*0.33; % 90% correction to fit the data


CMDmotorLeft=(CMDmotorLeft-1.1)/(1.9-1.1)*100;
CMDmotorRight=(CMDmotorRight-1.1)/(1.9-1.1)*100;

[A,B,C,D]=tf2ss([12.56],[1 12.56]);
sys_motor=ss(A,B,C,D);
f0L=14; %ESCLeft(1)/gr;
fL_cmd=CMDmotorLeft*s1L+s2L;
fL=lsim(sys_motor,fL_cmd,time,f0L/C)';
    
f0R=15;%ESCRight(1)/gr;
fR_cmd=CMDmotorRight*s1R+s2R;
fR=lsim(sys_motor,fR_cmd,time,f0R/C)';

figure
hold on
plot_rectangles(indexUP,indexDOWN,timesUP-t0,timesDOWN-t0,time-t0,9,19)
plot(time-t0,freqLeft,'r'),hold on
% plot((time(n_trigLeft(1:end-1))+time(n_trigLeft(2:end)))/2,fLeft,'rx--')
plot(time-t0,freqRight,'b')
plot(time-t0,fL,'r--')
plot(time-t0,fR,'b--')
% plot((time(n_trigRight(1:end-1))+time(n_trigRight(2:end)))/2,fRight,'bx--')
% plot(time,(CMDmotorLeft-min(CMDmotorLeft))/(max(CMDmotorLeft)-min(CMDmotorLeft))*(max(ESCLeft)-min(ESCLeft))/gr*1.2+0.9*min(ESCLeft)/gr,'k')
% plot(time,(CMDmotorRight-min(CMDmotorRight))/(max(CMDmotorRight)-min(CMDmotorRight))*max(ESCRight),'m')
% legend('Left wings ESC','Left wings Hall','Right wings ESC','Right wings Hall','Unscaled command left')
legend('Left wing-pair','Right wing-pair')
xlabel('time (s)')
ylabel('flapping frequency (Hz)')
set(gca,'Xlim',[-0.2,1.2])
set(gca,'Ylim',[9,19])
