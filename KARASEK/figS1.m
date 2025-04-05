close all
clear all
clc

addpath('force_balance_data')

load force_balance_thrust_double_wing.mat

cmdDW=throttle;
fDW=freqAvg;
thrustDW=-FzAvg;
UDW=UAvg;
IDW=IAvg;
fDW(1)=0;
thrustDW(1)=0;

AADW=-FzAvg/throttle;
CC_throttleDW=corrcoef(throttle,-FzAvg)

pDW=polyfit(freqAvg(2:end),-FzAvg(2:end),1);
yDW=polyval(pDW,freqAvg(2:end));
[fitDW gofDW]=fit(freqAvg(2:end)',-2*FzAvg(2:end)','poly1')
CC_freqDW=corrcoef(freqAvg(2:end),-FzAvg(2:end))

sDW=polyfit(cmdDW(2:end),freqAvg(2:end),1);
festDW=polyval(sDW,cmdDW(2:end));

load force_balance_thrust_single_wing.mat


cmdSW=throttle;
fSW=freqAvg;
thrustSW=-FzAvg;
USW=UAvg;
ISW=IAvg;
fSW(1)=0;
thrustSW(1)=0;

AASW=-2*FzAvg/throttle;
CC_throttleSW=corrcoef(throttle,-2*FzAvg)

pSW=polyfit(freqAvg(2:end),-2*FzAvg(2:end),1);
[fitSW gofSW]=fit(freqAvg(2:end)',-2*FzAvg(2:end)','poly1')
ySW=polyval(pSW,freqAvg(2:end));
CC_freqSW=corrcoef(freqAvg(2:end),-2*FzAvg(2:end))

sSW=polyfit(cmdSW(2:end),freqAvg(2:end),1);
festSW=polyval(sSW,cmdSW(2:end));



%% figures

m=0.02824;

figure
subplot(1,3,1)
plot(cmdSW,2*thrustSW,'bs-'), hold on
plot(cmdSW,AASW*cmdSW,'k--')
plot(cmdSW([1,end]),m*9.81*[1 1],'k:')
plot(m*9.81/AASW*[1 1],[-0.05 0.45],'k:')
% plot(cmdDW,thrustDW,'rx-'), hold on
% plot(cmdDW,AADW*cmdDW,'r--')
ylabel('thrust (N)')
xlabel('throttle command (%)')
ylim([0 0.45])

legend('2 * single wingpair',['y = ' num2str((AASW),'%.2e') '*x'],'operating point (no payload)')



subplot(1,3,2)
% figure
% subplot(2,1,1)
plot(fSW,2*thrustSW,'bs-'),hold on
plot(fDW,thrustDW,'ro-'),hold on
plot(fSW(2:end),ySW,'k--')
plot(fSW([1,end]),m*9.81*[1 1],'k:')
plot((m*9.81-pSW(2))/pSW(1)*[1 1],[-0.05 0.45],'k:')
legend('2 * single wingpair','complete robot',['y = ' num2str(pSW(1),'%.2e') '*x + ' num2str(pSW(2),'%.2e')],'operating point (no payload)')

ylabel('thrust (N)')
xlabel('flapping frequency (Hz)')
ylim([0 0.45])

subplot(1,3,3)
% subplot(2,1,2)
plot(fSW,thrustSW./(USW.*ISW),'bs-'), hold on
plot(fDW,thrustDW./(UDW.*IDW),'ro-'), hold on
plot((m*9.81-pSW(2))/pSW(1)*[1 1],[0 0.1],'k:')
ylabel('thrust to power (N/W)')
xlabel('flapping frequency (Hz)')
legend('flapping mechanism','complete robot','operating point (no payload)')

