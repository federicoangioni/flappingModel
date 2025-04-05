close all
clear all
clc

addpath('force_balance_data')

load force_balance_pitch.mat


%% linear fit
p=polyfit(pitch(1:end),MyAvg(1:end),1);
y=polyval(p,pitch(1,:));
CC=corrcoef(pitch(1:end),MyAvg(1:end))
[fit gof]=fit(pitch(1:end)',MyAvg(1:end)','poly1')


%% 
figure('pos',[10 110 250 350])
subplot(2,1,1)
hold on
errorbar(mean(pitch,1),mean(MyAvg,1),std(MyAvg,1))
plot(mean(pitch,1),y,'k--')
ylabel('pitch torque (Nmm)')
set(gca,'Xlim',[-105 105])
set(gca,'Ylim',[-5 5])

subplot(2,1,2)
hold on
errorbar(mean(pitch,1),-mean(FzAvg,1),std(FzAvg,1))
xlabel('pitch command (%)')
ylabel('thrust (N)')
set(gca,'Xlim',[-105 105])
set(gca,'Ylim',[0.205 0.225])
