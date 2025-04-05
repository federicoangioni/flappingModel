close all
clear all

addpath('support_files')
load dataset_revision.mat

exp_title='RP_RATIO';
NEXP=[57 54 58 55 56];
legend_list={'q/p 2.06','q/p 1.34', 'q/p 0.84', 'q/p 0.50', 'q/p 0.35'};
cmdROLL=[1500 2000 2500 3000 3500];
cmdPITCH=[8000 6400 4800 3200 2000];

colors=viridis(170); % 30 to 200

%% initialize variables
deltaHEADmaxAvg=[];
deltaHEADmaxStd=[];
deltaHEADendAvg=[];
deltaHEADendStd=[];

dHEADmeanAvg=[];
dHEADmeanStd=[];
dHEADmaxAvg=[];
dHEADmaxStd=[];

deltaYAWmaxAvg=[];
deltaYAWmaxStd=[];
deltaYAWendAvg=[];
deltaYAWendStd=[];

PITCH2ROLLavg=[];
PITCH2ROLLstd=[];
TIMEMAN=[];

OMxEndOLavg=[];
OMxEndOLstd=[];
OMyEndOLavg=[];
OMyEndOLstd=[];

psiOMendOLavg=[];
psiOMendOLstd=[];

deltaHEADall=[];
dHEADall=[];
pitch2rollall=[];

%% loop through all experiments

for i=1:length(NEXP)
    Nexp=NEXP(i);
    
    eval(['data=experiment' num2str(Nexp) ';'])
    assign_repetition_variables
    
    Nman=size(ROLL,1);
    
    % compute the heading and body yaw change
    deltaHEAD=HEADaligned-repmat(HEADaligned(:,TIME==0),1,length(HEADaligned_avg));
    deltaYAW=YAWaligned-repmat(YAWaligned(:,TIME==0),1,length(HEADaligned_avg));
    
    % consider only heading and sideslip within 1 second after the end of OL
    TF=1+TIMEman;
    TF=0.6;
    TF=0.8;
    TS=-0.1;
        
    OM=(OMx.^2+OMy.^2+OMz.^2).^0.5;
    if Nman>1
        OMavg=mean(OM);
    else
        OMavg=OM;
    end
    
   	ALPH=(ALPHx.^2+ALPHy.^2+ALPHz.^2).^0.5;
    ALPHavg=mean(ALPH);
     
    % data at the end of the OL part
    TIMEendOL=TIME(TIME>TIMEman);
    TIMEendOL=TIMEendOL(1);
    OMxEndOL=OMx(:,TIME==TIMEendOL);
    OMyEndOL=OMy(:,TIME==TIMEendOL);

    pitch2roll_endOL=OMyEndOL./OMxEndOL;
    
    pitch2roll=max(OMyavg(1.5*120:(1.5+TF)*120))/max(OMxavg(1.5*120:(1.5+TF)*120));
    PITCH2ROLLavg=[PITCH2ROLLavg pitch2roll];
    

    
    f_trans=2*17;
    R_trans=0.14;

    figure(1)
    ax_all(1)=subplot(3,3,1); hold on
    plot(TIME*f_trans,OMxavg/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    ylabel('p (halfwingb.^-^1)')
    set(gca,'Ylim',[-13 13])
    set(gca,'Xlim',[0 15])
    
    ax_all(4)=subplot(3,3,4); hold on
    plot(TIME*f_trans,OMyavg/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';

    ylabel('q (halfwingb.^-^1)')
    set(gca,'Ylim',[-13 13])
    set(gca,'Xlim',[0 15])
    
    ax_all(7)=subplot(3,3,7); hold on
    plot(TIME*f_trans,OMzavg/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    ylabel('r (halfwingb.^-^1)')
    xlabel('halfwingbeats')
    set(gca,'Ylim',[-13 13])
    set(gca,'Xlim',[0 15])
    
%     figure(2)
    ax_all(2)=subplot(3,3,2); hold on
    plot(TIME*f_trans,VELxavg/R_trans/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ylabel('u (R/halfwingb.)')
    set(gca,'Ylim',[-0.1 0.5])
    set(gca,'Xlim',[0 15])
    
    ax_all(5)=subplot(3,3,5); hold on
    plot(TIME*f_trans,VELyavg/R_trans/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ylabel('v (R/halfwingb.)')
    set(gca,'Ylim',[-0.1 0.5])
    set(gca,'Xlim',[0 15])
    
    ax_all(8)=subplot(3,3,8); hold on
    plot(TIME*f_trans,VELzavg/R_trans/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ylabel('w (R/halfwingb.)')
    xlabel('halfwingbeats')
	set(gca,'Ylim',[-0.5 0.1])
    set(gca,'Xlim',[0 15])


%     figure(3)
    ax_all(3)=subplot(3,3,3); hold on
    plot(TIME*f_trans,BETavg-BETavg(1.5*120),'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    ylabel('\Delta sideslip')
    set(gca,'Ylim',[-20 100])
    set(gca,'Xlim',[0 15])
    
    ax_all(6)=subplot(3,3,6); hold on
    plot(TIME*f_trans,HEADaligned_avg-HEADaligned_avg(1.5*120),'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    ylabel('\Delta course')
    set(gca,'Ylim',[-20 180])
    set(gca,'Xlim',[0 15])
    
    ax_all(9)=subplot(3,3,9); hold on
    plot(TIME*f_trans,(VELOCavg-VELOCavg(1.5*120))/R_trans/f_trans,'Color',colors(200-round(pitch2roll*100),:),'LineWidth',2)
%     grid on
    ylabel('\Delta U (R/halfwingb.)')
    xlabel('halfwingbeats')
    ylim([-0.25 0.35])
    set(gca,'Xlim',[0 15])
    
end


