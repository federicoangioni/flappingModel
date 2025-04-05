close all
clear all

addpath('support_files')
load dataset_revision.mat

exp_title='RP_RATIO';
NEXP=[57 54 58 55 56];
legend_list={'q/p 2.06','q/p 1.34', 'q/p 0.84', 'q/p 0.50', 'q/p 0.35'};
cmdROLL=[1500 2000 2500 3000 3500];
cmdPITCH=[8000 6400 4800 3200 2000];

colors=viridis(length(NEXP));
colors_light=colors;

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
    TF=0.8;
    TS=-0.1;
    
    TIME1=TIME(TIME<=TF);
    select1=zeros(size(TIME));
    select2=zeros(size(TIME));
    select1(TIME<=TF)=1;
    select2(TIME>=TS)=1;
    select=select1.*select2;
    
    POSx1=POSx(TIME<=TF);
    POSx1=POSx1(TIME1>=TS);
    POSy1=POSy(TIME<=TF);
    POSy1=POSy1(TIME1>=TS);
    POSz1=POSz(TIME<=TF);
    POSz1=POSz1(TIME1>=TS);
    deltaHEAD=deltaHEAD(:,TIME<=TF);
    deltaHEAD=deltaHEAD(:,TIME1>=0);
    dHEAD1=dHEAD(:,TIME<=TF);
    dHEAD1=dHEAD1(:,TIME1>=0);
    deltaYAW=deltaYAW(:,TIME<=TF);
    deltaYAW=deltaYAW(:,TIME1>=0);
    BET1=BET(:,TIME<=TF);
    BET1=BET1(:,TIME1>=0);
    OMx1=OMx(:,TIME<=TF);
    OMx1=OMx1(:,TIME1>=0);
    OMy1=OMy(:,TIME<=TF);
    OMy1=OMy1(:,TIME1>=0);
    OMz1=OMz(:,TIME<=TF);
    OMz1=OMz1(:,TIME1>=0);
    VELOC1=VELOC(:,TIME<=TF);
    VELOC1=VELOC1(:,TIME1>=0);
    VELOC1N=VELOC1./repmat(VELOC1(:,1),1,size(VELOC1,2));
    
    dHEAD2=dHEAD(:,TIME<=0.5);
    TIME2=TIME(TIME<=0.5);
    dHEAD2=dHEAD2(:,TIME2>=0);
    
    OM=(OMx.^2+OMy.^2+OMz.^2).^0.5;
    thetaOM=atan2(OMz,(OMx.^2+OMy.^2).^0.5);
    psiOM=atan2(OMy,OMx);
    psiOM(psiOM<0)=psiOM(psiOM<0)+2*pi;
    
    if Nman>1
        OMavg=mean(OM);
    else
        OMavg=OM;
    end
    thetaOMavg=atan2(OMzavg,(OMxavg.^2+OMyavg.^2).^0.5);
    psiOMavg=atan2(OMyavg,OMxavg);
    psiOMavg(psiOMavg<0)=psiOMavg(psiOMavg<0)+2*pi;
    
    
    
    deltaHEADmaxAvg=[deltaHEADmaxAvg mean(max(deltaHEAD'))];
    deltaHEADmaxStd=[deltaHEADmaxStd std(max(deltaHEAD'))];
    deltaHEADendAvg=[deltaHEADendAvg mean(deltaHEAD(:,end)')];
    deltaHEADendStd=[deltaHEADendStd std(deltaHEAD(:,end)')];
    
    deltaYAWmaxAvg=[deltaYAWmaxAvg mean(max(deltaYAW'))];
    deltaYAWmaxStd=[deltaYAWmaxStd std(max(deltaYAW'))];
    deltaYAWendAvg=[deltaYAWendAvg mean(deltaYAW(:,end)')];
    deltaYAWendStd=[deltaYAWendStd std(deltaYAW(:,end)')];
    
    % data at the end of the OL part
    TIMEendOL=TIME(TIME>TIMEman);
    TIMEendOL=TIMEendOL(1);
    OMxEndOL=OMx(:,TIME==TIMEendOL);
    OMyEndOL=OMy(:,TIME==TIMEendOL);
    OMxEndOLavg=[OMxEndOLavg mean(OMx(:,TIME==TIMEendOL))];
    OMxEndOLstd=[OMxEndOLstd std(OMx(:,TIME==TIMEendOL))];
    OMyEndOLavg=[OMyEndOLavg mean(OMy(:,TIME==TIMEendOL))];
    OMyEndOLstd=[OMyEndOLstd std(OMy(:,TIME==TIMEendOL))];
    
    psiOMendOLavg=[psiOMendOLavg mean(psiOM(:,TIME==TIMEendOL))];
    psiOMendOLstd=[psiOMendOLstd std(psiOM(:,TIME==TIMEendOL))];
    
    %     pitch2roll=max(OMy1')./max(OMx1');
    pitch2roll=OMyEndOL./OMxEndOL;
    
    deltaHEADall=[deltaHEADall max(deltaHEAD')];
    dHEADall=[dHEADall max(dHEAD2')];
    pitch2rollall=[pitch2rollall pitch2roll'];
    
    
    % indices of max heading changes
    [~,deltaHEADmaxInd]=max(deltaHEAD');
    
    dHEADmean=[];
    for j=1:Nman
        dHEADmean=[dHEADmean mean(dHEAD1(j,1:deltaHEADmaxInd(j)))];
    end
    
    dHEADmeanAvg=[dHEADmeanAvg mean(dHEADmean')];
    dHEADmeanStd=[dHEADmeanStd std(dHEADmean')];
    dHEADmaxAvg=[dHEADmaxAvg mean(max(dHEAD2'))];
    dHEADmaxStd=[dHEADmaxStd std(max(dHEAD2'))];
    
    PITCH2ROLLavg=[PITCH2ROLLavg mean(pitch2roll)];
    PITCH2ROLLstd=[PITCH2ROLLstd std(pitch2roll)];
    TIMEMAN=[TIMEMAN TIMEman];
    
if i==1
        figure('Position',[100 100 1150 520])
    end
    
    ax(1)=subplot(4,5,1); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,CMDroll,'Color',colors_light(i,:))
    plot(TIME,CMDrollavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('roll cmd (%)')
    set(gca,'Ylim',[-100 100])
    set(gca,'XColor','none'); 
        
    ax(6)=subplot(4,5,6); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,CMDpitch,'Color',colors_light(i,:))
    plot(TIME,CMDpitchavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('pitch cmd (%)')
    set(gca,'Ylim',[-100 100])
    set(gca,'XColor','none'); 
        
    ax(11)=subplot(4,5,11); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,CMDyaw,'Color',colors_light(i,:))
    plot(TIME,CMDyawavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw cmd (%)')
    set(gca,'Ylim',[-100 100])
    set(gca,'XColor','none'); 
    
    ax(16)=subplot(4,5,16); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,CMDthrust,'Color',colors_light(i,:))
    plot(TIME,CMDthrustavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('throttle cmd (%)')
    set(gca,'Ylim',[0 100])
    xlabel('time (s)')
     
    
    
    
    ax(2)=subplot(4,5,2); hold on
    if i==1
        rectangle('Position',[0 -180 TIMEman 360],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,ROLL,'Color',colors_light(i,:))
    eval(['line_rpy' num2str(i) '=plot(TIME,ROLLavg,''Color'',colors(i,:),''LineWidth'',2);'])
    ylabel('roll')
    set(gca,'Ylim',[-15 100])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none'); 
    
    ax(7)=subplot(4,5,7); hold on
    if i==1
        rectangle('Position',[0 -180 TIMEman 360],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,PITCH,'Color',colors_light(i,:))
    plot(TIME,PITCHavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('pitch')
    set(gca,'Ylim',[-50 60])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(12)=subplot(4,5,12); hold on
    if i==1
        rectangle('Position',[0 -180 TIMEman 360],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,YAWaligned,'Color',colors_light(i,:))
    plot(TIME,YAWaligned_avg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw')
    xlabel('time (s)')
    set(gca,'Ylim',[-10 150])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(17)=subplot(4,5,17); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,HEADaligned,'Color',colors_light(i,:))
    plot(TIME,HEADaligned_avg,'Color',colors(i,:),'LineWidth',2)
    ylabel('course')
    set(gca,'Ylim',[-10 200])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    xlabel('time (s)')
        
    
    ax(3)=subplot(4,5,3); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMx,'Color',colors_light(i,:))
    plot(TIME,OMxavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('roll rate (s^-^1)')
    set(gca,'Ylim',[-500 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
        
    ax(8)=subplot(4,5,8); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMy,'Color',colors_light(i,:))
    plot(TIME,OMyavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('pitch rate (s^-^1)')
    set(gca,'Ylim',[-500 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(13)=subplot(4,5,13); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMz,'Color',colors_light(i,:))
    plot(TIME,OMzavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw rate (s^-^1)')
    set(gca,'Ylim',[-500 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(18)=subplot(4,5,18); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,dHEAD,'Color',colors_light(i,:))
    plot(TIME,dHEADavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('turn rate (s^-^1)')
    set(gca,'Ylim',[-100 1500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    xlabel('time (s)')
    
    
    ax(4)=subplot(4,5,4); hold on
    if i==1
        rectangle('Position',[0 -10000 TIMEman 20000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,ALPHx,'Color',colors_light(i,:))
    plot(TIME,ALPHxavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('roll acc. (s^-^2)')
    set(gca,'Ylim',[-7000 7000])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(9)=subplot(4,5,9); hold on
    if i==1
        rectangle('Position',[0 -10000 TIMEman 20000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,ALPHy,'Color',colors_light(i,:))
    plot(TIME,ALPHyavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('pitch acc. (s^-^2)')
    set(gca,'Ylim',[-5000 5000])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
        
    ax(14)=subplot(4,5,14); hold on
    if i==1
        rectangle('Position',[0 -10000 TIMEman 20000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,ALPHz,'Color',colors_light(i,:))
    plot(TIME,ALPHzavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw acc. (s^-^2)')
    xlabel('time (s)')
    set(gca,'Ylim',[-5000 5000])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
        
    ax(19)=subplot(4,5,19); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,VELOC,'Color',colors_light(i,:))
    plot(TIME,VELOCavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('speed (m.s^-^1)')
    xlabel('time (s)')
    set(gca,'Ylim',[0 2.5])
    

    
    ax(5)=subplot(4,5,5); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,FREQ,'Color',colors_light(i,:))
    plot(TIME,FREQavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('right w. freq. (Hz)')
    set(gca,'Ylim',[10 25])
    set(gca,'XColor','none');
    
    ax(10)=subplot(4,5,10); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,DIHED,'Color',colors(i,:))
%     plot(TIME,DIHEDavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('dihedral')
    set(gca,'Ylim',[-35 25])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(15)=subplot(4,5,15); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,BET,'Color',colors_light(i,:))
    plot(TIME,BETavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('sideslip')
    xlabel('time (s)')
    set(gca,'Ylim',[-20 90])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
end
   
linkaxes(ax,'x')
set(gca,'Xlim',[-0.1 1.5])

if strcmp(exp_title,'RP_RATIO')
    set(gca,'Xlim',[-0.1 0.6])
    Nwingbeats=0.7*17;
elseif strcmp(exp_title,'FIG4 combined3')
    set(gca,'Xlim',[-0.1 0.8])
    Nwingbeats=0.9*17;
else
    Nwingbeats=4.5*17;
end

for i=1:5
    subplot(4,5,i);
    wingbeat_axis_top(gca,Nwingbeats,1)
end

for i=1:length(NEXP)
    if i==1
        legend_rpy='line_rpy1';
    else
        legend_rpy=[legend_rpy ',line_rpy' num2str(i)];
    end
end

eval(['legend([' legend_rpy '],legend_list)'])
