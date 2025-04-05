close all
clear all
clc

addpath('force_balance_data')

load force_balance_roll.mat

%% linear fit
p=polyfit(roll(1:end),MxAvg(1:end),1);
y=polyval(p,roll(1,:));
CC=corrcoef(roll(1:end),MxAvg(1:end))
[fit gof]=fit(roll(1:end)',MxAvg(1:end)','poly1')


%% 
figure('pos',[10 110 250 350])
subplot(2,1,1)
hold on
errorbar(mean(roll,1),mean(MxAvg,1),std(MxAvg,1))
plot(mean(roll,1),y,'k--')
ylabel('roll torque (Nmm)')
set(gca,'Xlim',[-31 31])
set(gca,'Ylim',[-8.5 8.5])

subplot(2,1,2)
hold on
errorbar(mean(roll,1),-mean(FzAvg,1),std(FzAvg,1))
xlabel('roll command (%)')
ylabel('thrust (N)')
set(gca,'Xlim',[-31 31])
set(gca,'Ylim',[0.198 0.218])
