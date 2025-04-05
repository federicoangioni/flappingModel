close all
clear all

addpath('fly_data')

import_and_sort_fly_data

U=[];
V=[];
W=[];

P=[];
Q=[];
R=[];

PD=[];
QD=[];
RD=[];

%% correlation between torques


for jj=1:4
    eval(['U=mean(VELX' num2str(jj) ',2,''omitnan'');'])
    eval(['V=mean(VELY' num2str(jj) ',2,''omitnan'');'])
    eval(['W=mean(VELZ' num2str(jj) ',2,''omitnan'');'])
    eval(['P=mean(OMX' num2str(jj) ',2,''omitnan'');'])
    eval(['Q=mean(OMY' num2str(jj) ',2,''omitnan'');'])
    eval(['R=mean(OMZ' num2str(jj) ',2,''omitnan'');'])
    eval(['PD=mean(ALPHX' num2str(jj) ',2,''omitnan'');'])
    eval(['QD=mean(ALPHY' num2str(jj) ',2,''omitnan'');'])
    eval(['RD=mean(ALPHZ' num2str(jj) ',2,''omitnan'');'])
    eval(['Pnresp=OMX' num2str(jj) '(301,:);'])
    eval(['Qnresp=OMY' num2str(jj) '(301,:);'])
    
    c6=corrcoef(RD,PD);
    c7=corrcoef(RD,QD);
%     c8=corrcoef(RD,PD.*U);
%     c9=corrcoef(RD,QD.*V);
%     c10=corrcoef(RD+Cyaw_fly*R/Izz_fly,PD);
%     c11=corrcoef(RD+Cyaw_fly*R/Izz_fly,QD);
    c12=corrcoef(RD+Cyaw_fly*R/Izz_fly,PD.*U);
    c13=corrcoef(RD+Cyaw_fly*R/Izz_fly,QD.*V);
%     c14=corrcoef(RD+Cyaw_fly*R/Izz_fly,(PD+Croll_fly*P/Ixx_fly).*U);
%     c15=corrcoef(RD+Cyaw_fly*R/Izz_fly,(QD+Cpitch_fly*Q/Iyy_fly).*V);
    
    
    c_dpdr(jj)=c6(2,1);
    c_dqdr(jj)=c7(2,1);
%     c_dpudr(jj)=c8(2,1);
%     c_dqvdr(jj)=c9(2,1);
%     c_dpdrFCT(jj)=c10(2,1);
%     c_dqdrFCT(jj)=c11(2,1);
    c_dpudrFCT(jj)=c12(2,1);
    c_dqvdrFCT(jj)=c13(2,1);
%     c_dpudrFCT2(jj)=c14(2,1);
%     c_dqvdrFCT2(jj)=c15(2,1);
    
    q2p(jj)=max(Q)/max(P);
    q2p_nresp(jj)=mean(Qnresp./Pnresp);
    
    time_fly=TIME(:,1);
    
end

% q2p_fly=q2p_nresp;
q2p_fly=q2p;
c_dpdr_fly=c_dpdr;
c_dqdr_fly=c_dqdr;
% c_dpudr_fly=c_dpudr;
% c_dqvdr_fly=c_dqvdr;
% c_dpdrFCT_fly=c_dpdrFCT;
% c_dqdrFCT_fly=c_dqdrFCT;
c_dpudrFCT_fly=c_dpudrFCT;
c_dqvdrFCT_fly=c_dqvdrFCT;
% c_dpudrFCT2_fly=c_dpudrFCT2;
% c_dqvdrFCT2_fly=c_dqvdrFCT2;

%%

figure,
plot(q2p_fly,c_dpdr_fly,'b-d','LineWidth',2), hold on
plot(q2p_fly,c_dqdr_fly,'b--s','LineWidth',2)
legend('dp','dq')
title('correlation with dr/dt')
xlabel('max(q)/max(p)')
ylabel('corr. coeff.')
ylim([-1 -0.2])
xlim([0.2 2])

figure,
plot(q2p_fly,c_dpudrFCT_fly,'b-+','LineWidth',2), hold on
plot(q2p_fly,c_dqvdrFCT_fly,'b--x','LineWidth',2)
legend('dp*u','dq*v')
title('correlation with [dr/dt + FCT]')
xlabel('max(q)/max(p)')
ylabel('corr. coeff.')
ylim([-1 -0.2])
xlim([0.2 2])
