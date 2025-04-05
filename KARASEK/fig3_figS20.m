close all
clear all

addpath('support_files')
addpath('support_files/errorbarxy')
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
    TF=0.6;
    %     TF=0.8;
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
    
    % Repetition figures
    figure(1);
    
    hold on
    plot3(POSx(:,select==1)',-POSy(:,select==1)',-POSz(:,select==1)','Color',colors_light(i,:))
    eval(['line_pos' num2str(i) '=plot3(POSxavg(:,select==1),-POSyavg(:,select==1),-POSzavg(:,select==1),''Color'',colors(i,:),''LineWidth'',2);'])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    view([0,0,1])
    axis([-1.5 1 -2.5 0.5 -1 1])
    axis equal
    
%     figure(2)
%     ax_head(1)=subplot(4,1,1); hold on
%     if i==1
%         rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
%     end
%     plot(TIME,dHEAD,'Color',colors_light(i,:))
%     plot(TIME,dHEADavg,'Color',colors(i,:),'LineWidth',2)
%     ylabel('turn rate (s^-^1)')
%     set(gca,'Ylim',[-100 1500])
%     ca=gca;
%     ca.YAxis.TickLabelFormat = '%g\\circ';
%     
%     ax_head(2)=subplot(4,1,2); hold on
%     if i==1
%         rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
%     end
%     plot(TIME,HEAD,'Color',colors_light(i,:))
%     plot(TIME,HEADavg,'Color',colors(i,:),'LineWidth',2)
%     ylabel('course')
%     set(gca,'Ylim',[-30 180])
%     ca=gca;
%     ca.YAxis.TickLabelFormat = '%g\\circ';
%     
%     ax_head(3)=subplot(4,1,3); hold on
%     if i==1
%         rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
%     end
%     plot(TIME,BET,'Color',colors_light(i,:))
%     plot(TIME,BETavg,'Color',colors(i,:),'LineWidth',2)
%     ylabel('sideslip')
%     set(gca,'Ylim',[-120 120])
%     ca=gca;
%     ca.YAxis.TickLabelFormat = '%g\\circ';
%     
%     ax_head(4)=subplot(4,1,4); hold on
%     if i==1
%         rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
%     end
%     plot(TIME,VELOC,'Color',colors_light(i,:))
%     plot(TIME,VELOCavg,'Color',colors(i,:),'LineWidth',2)
%     ylabel('v (m/s)')
%     xlabel('time (s)')
%     set(gca,'Ylim',[0 3])
    
    figure(3)
    ax_om(1)=subplot(3,1,1); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMx,'Color',colors_light(i,:))
    plot(TIME,OMxavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('p (s^-^1)')
    set(gca,'Ylim',[-700 700])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
    ax_om(2)=subplot(3,1,2); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMy,'Color',colors_light(i,:))
    plot(TIME,OMyavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('q (s^-^1)')
    set(gca,'Ylim',[-500 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
    ax_om(3)=subplot(3,1,3); hold on
    if i==1
        rectangle('Position',[0 -2000 TIMEman 4000],'FaceColor',[0.85 0.85 0.85],'EdgeColor','none')
    end
    plot(TIME,OMz,'Color',colors_light(i,:))
    plot(TIME,OMzavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('r (s^-^1)')
    xlabel('time (s)')
    set(gca,'Ylim',[-500 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
    figure(4)
    hold on
    plot(pitch2roll,max(deltaHEAD'),'ks','MarkerFaceColor',colors(i,:))
    ylabel('course change')
    xlabel('q/p (-)')
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
    figure(5)
    hold on
    plot(pitch2roll,max(dHEAD2'),'ks','MarkerFaceColor',colors(i,:))
    
    figure(6)
    if i==1
        polarplot([0; 0]/180*pi,[0,100],'k--','LineWidth',2)
        hold on
    end
    polarplot([psiOM(:,TIME==0.25)'; psiOM(:,TIME==0.25)'], [zeros(size(OM(:,TIME==0.25)')); OM(:,TIME==0.25)'],'Color',colors(i,:))
    polarplot([psiOMavg(TIME==0.25); psiOMavg(TIME==0.25)], [0; OMavg(TIME==0.25)],'Color',colors(i,:),'LineWidth',2)
    set(gca,'ThetaDir','clockwise','ThetaZeroLocation','right')
    
    title('Rotation axis')
    %     rlim([0 150])
    
end

% link all axes together
set(ax_om,'Xlim',[-0.1 0.6])


%% add legends

slopeROLL=OMxEndOLavg/cmdROLL
slopePITCH=OMyEndOLavg/cmdPITCH

PITCH2ROLLcmd=(cmdPITCH*slopePITCH)./(cmdROLL*slopeROLL)

for i=1:length(NEXP)
    if i==1
        legend_pos='line_pos1';
        legend_rpy='line_rpy1';
    else
        legend_pos=[legend_pos ',line_pos' num2str(i)];
        legend_rpy=[legend_rpy ',line_rpy' num2str(i)];
    end
end

figure(1)
eval(['legend([' legend_pos '],legend_list)'])
xlim([-.2 .55])
ylim([-.6 .1])

addpath('\errorbarxy')

figure(4)
errorbarxy(PITCH2ROLLavg,deltaHEADmaxAvg,PITCH2ROLLstd,deltaHEADmaxStd,{'k.-', 'k', 'k'})
set(gca,'Xlim',[0.2,2.6])
set(gca,'Ylim',[70,200])

figure(5)
hold on
errorbarxy(PITCH2ROLLavg,dHEADmaxAvg,PITCH2ROLLstd,dHEADmaxStd,{'k.-', 'k', 'k'})
set(gca,'Xlim',[0.2,2.6])
ylabel('turn rate (deg/s)')
xlabel('q/p (-)')
set(gca,'Ylim',[300,2100])

%% Figure S20
figure
subplot(2,1,1)
errorbar(cmdROLL/9600*100,OMxEndOLavg,OMxEndOLstd),hold on
plot([0 cmdROLL]/9600*100,[0 cmdROLL*slopeROLL],'r--')
ylabel('roll rate (s^-^1)')
xlabel('roll command (%)')
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
box off

legend('Measurement',['y = ' num2str(slopeROLL,'%.1e') '*x'])

subplot(2,1,2)
errorbar(cmdPITCH/9600*100,OMyEndOLavg,OMyEndOLstd),hold on
plot([cmdPITCH 0]/9600*100,[cmdPITCH*slopePITCH 0],'r--')
ylabel('pitch rate (s^-^1)')
xlabel('pitch command (%)')
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
box off

legend('Measurement',['y = ' num2str(slopePITCH,'%.1e') '*x'])

%%

corrcoef(pitch2rollall,deltaHEADall)
corrcoef(pitch2rollall,dHEADall)
corrcoef(deltaHEADall,dHEADall)



