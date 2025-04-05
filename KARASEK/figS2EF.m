close all
clear all
clc

addpath('force_balance_data')

load force_balance_yaw.mat


%% linear fit
p=polyfit(yaw(1:end),MzAvg(1:end),1);
y=polyval(p,yaw(1,:));
CC=corrcoef(yaw(1:end),MzAvg(1:end))
[fit gof]=fit(yaw(1:end)',MzAvg(1:end)','poly1')
% [fit gof]=fit(mean(yaw,1)',mean(MzAvg,1)','poly1')


%% 
figure('pos',[10 110 250 350])
subplot(2,1,1)
hold on
errorbar(mean(yaw,1),mean(MzAvg,1),std(MzAvg,1))
plot(mean(yaw,1),y,'k--')
ylabel('yaw torque (Nmm)')
set(gca,'Xlim',[-105 105])
set(gca,'Ylim',[-3.5 3.5])

subplot(2,1,2)
hold on
errorbar(mean(yaw,1),-mean(FzAvg,1),std(FzAvg,1))
xlabel('yaw command (%)')
ylabel('thrust (N)')
set(gca,'Xlim',[-105 105])
set(gca,'Ylim',[0.21 0.23])


