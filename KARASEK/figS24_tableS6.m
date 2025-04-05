close all
clear all
clc

addpath('support_files')
load dataset_revision.mat

reduced_model_prepare_data

for i=1:5
    model_err=RD(:,i)+bx*((FL(:,i)+FR(:,i)).*ly.^2.*R(:,i) + (FL(:,i)-FR(:,i)).*ly.*U(:,i) + (FL(:,i)+FR(:,i)).*ly.*V(:,i).*DIHED_MEAS(:,i))/Izz;
    model_rmse(i)=sqrt(mean(model_err.^2,'omitnan'));
    model_rmse_norm(i)=sqrt(mean(model_err.^2,'omitnan'))/max(abs(RD(:,i)));
    
    q2p_max(i)=max(Q(:,i))/max(P(:,i));
    % q2p_endOL(i)=Q(31,i)/P(31,i);
    
    rd_reduced=-bx*((FL(~isnan(RD(:,i)),i)+FR(~isnan(RD(:,i)),i)).*ly.^2.*R(~isnan(RD(:,i)),i) + (FL(~isnan(RD(:,i)),i)-FR(~isnan(RD(:,i)),i)).*ly.*U(~isnan(RD(:,i)),i) + (FL(~isnan(RD(:,i)),i)+FR(~isnan(RD(:,i)),i)).*ly.*V(~isnan(RD(:,i)),i).*DIHED_MEAS(~isnan(RD(:,i)),i))/Izz;
    
    RDa=RD(~isnan(RD(:,i)),i);
    corrR=corrcoef(RDa(~isnan(rd_reduced)),rd_reduced(~isnan(rd_reduced)));
    c_model(i)=corrR(2,1);
    
    
    figure('Position',[100 100 275 120])
    % subplot(2,1,1)
    rectangle('Position',[0 -7000 0.25 13000],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
    plot(TIME,RD(:,i)/pi*180,'r','LineWidth',2), hold on
    plot(TIME,-bx*((FL(:,i)+FR(:,i)).*ly.^2.*R(:,i))/Izz/pi*180,'b');
    plot(TIME,-bx*((FL(:,i)-FR(:,i)).*ly.*U(:,i))/Izz/pi*180,'m');
    plot(TIME,-bx*((FL(:,i)+FR(:,i)).*ly.*V(:,i).*DIHED_MEAS(:,i))/Izz/pi*180,'g');
    plot(TIME,-bx*((FL(:,i)+FR(:,i)).*ly.^2.*R(:,i) + (FL(:,i)-FR(:,i)).*ly.*U(:,i) + (FL(:,i)+FR(:,i)).*ly.*V(:,i).*DIHED_MEAS(:,i))/Izz/pi*180,'k','LineWidth',2);
    % title(['Original b, corr. coeff. ' num2str(corrR(2,1)) ', rmse ' num2str(model_rmse(i)/pi*180) ' deg/s^2'])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    ylabel('dr/dt (deg/s^2)')
    grid
    % legend('exp.','FCT','roll','pitch','total','location','eastoutside')
    legend('exp.','FCT','roll','pitch','total')
    xlim([0 0.6])
    ylim([-7000 6000])
    
    % saveas(gcf,['set' num2str(i) '_model_components.fig'])
    % saveas(gcf,['set' num2str(i) '_model_components.svg'])
    % saveas(gcf,['set' num2str(i) '_model_components.png'])
    
    figure('Position',[100 100 275 120])
    rectangle('Position',[0 5 0.25 20],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
    plot(TIME,FL(:,i),'b'), hold on
    plot(TIME,FR(:,i),'r'), hold on
    plot(TIME,FR_MEAS(:,i),'r--'), hold on
    ylabel('f (Hz)')
    grid
    legend('model left','model right','exp. right')
    xlim([0 0.6])
    ylim([11.5 22.5])
    
    % saveas(gcf,['set' num2str(i) '_freq.fig'])
    % saveas(gcf,['set' num2str(i) '_freq.svg'])
    % saveas(gcf,['set' num2str(i) '_freq.png'])
end

figure('Position',[100 100 275 120])
plot(0,0)
xlim([0 0.6*17])
set(gca,'XTick',[0:1:15])

c_model

