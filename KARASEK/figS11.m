close all
clear all

addpath('support_files')
load dataset_revision.mat

exp_title='PITCH_STEPS';
NEXP=[102 101 103 94 104];
legend_list={'pitch 15\circ','pitch 30\circ','pitch 45\circ','pitch 60\circ','pitch 75\circ'};
cmdROLL=[0 0 0 0 0];
cmdPITCH=[1920 3840 5760 7680 9600];

colors=viridis(length(NEXP));
colors_light=colors;

nSigma=1; % number of sigmas to show in error bands


%% initialize variables
TIMEMAN=[];

deltaHEADall=[];
dHEADall=[];


%% loop through all experiments

for i=1:length(NEXP)
    Nexp=NEXP(i);
    
    eval(['data=experiment' num2str(Nexp) ';'])
    assign_repetition_variables
    
    Nman=size(ROLL,1);
    
    % compute the heading and body yaw change
    deltaHEAD=HEADaligned-repmat(HEADaligned(:,TIME==0),1,length(HEADaligned_avg));
    deltaYAW=YAWaligned-repmat(YAWaligned(:,TIME==0),1,length(HEADaligned_avg));
    
    OM=(OMx.^2+OMy.^2+OMz.^2).^0.5;
    if Nman>1
        OMavg=mean(OM);
        OMstd=std(OM);
    else
        OMavg=OM;
        OMstd=zeros(size(OM));
    end
    
    ALPH=(ALPHx.^2+ALPHy.^2+ALPHz.^2).^0.5;
    ALPHavg=mean(ALPH);
    ACCEL=(ACCx.^2+ACCy.^2+ACCz.^2).^0.5;
    ACCELavg=mean(ACCEL);
    ACCELstd=std(ACCEL);

    figure(1)
    ax(1)=subplot(3,4,1); hold on
    %     plot(TIME,ROLL,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[ROLLavg'+nSigma*ROLLstd';flipud(ROLLavg'-nSigma*ROLLstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    eval(['line_rpy' num2str(i) '=plot(TIME,ROLLavg,''Color'',colors(i,:),''LineWidth'',2);'])
    ylabel('roll')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-45 45])
    elseif strcmp(exp_title,'ROLL_STEPS')
        plot(TIME,RCrollavg,'k--','LineWidth',1)
        set(gca,'Ylim',[-75 15])
    end
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(5)=subplot(3,4,5); hold on
    %     plot(TIME,PITCH,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[PITCHavg'+nSigma*PITCHstd';flipud(PITCHavg'-nSigma*PITCHstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,PITCHavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('pitch')
    if strcmp(exp_title,'PITCH_STEPS')
        plot(TIME,RCpitchavg,'k--','LineWidth',1)
        set(gca,'Ylim',[-85 5])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-45 45])
    end
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(9)=subplot(3,4,9); hold on
    %     plot(TIME,YAW,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[YAWaligned_avg'+nSigma*YAWaligned_std';flipud(YAWaligned_avg'-nSigma*YAWaligned_std')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,YAWaligned_avg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw')
    xlabel('time (s)')
    set(gca,'Ylim',[-45 45])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    
    ax(2)=subplot(3,4,2); hold on
    %     plot(TIME,OMx,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[OMxavg'+nSigma*OMxstd';flipud(OMxavg'-nSigma*OMxstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,OMxavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('roll rate (s^-^1)')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-250 250])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-450 100])
    end
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(6)=subplot(3,4,6); hold on
    %     plot(TIME,OMy,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[OMyavg'+nSigma*OMystd';flipud(OMyavg'-nSigma*OMystd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    %     plot(TIME,OMyavg,'Color',colors(i,:),'LineWidth',2)
    eval(['line_rpy' num2str(i) '=plot(TIME,OMyavg,''Color'',colors(i,:),''LineWidth'',2);'])
    ylabel('pitch rate (s^-^1)')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-425 125])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-275 275])
    end
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(10)=subplot(3,4,10); hold on
    %     plot(TIME,OMz,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[OMzavg'+nSigma*OMzstd';flipud(OMzavg'-nSigma*OMzstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,OMzavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('yaw rate (s^-^1)')
    set(gca,'Ylim',[-275 275])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    xlabel('time (s)')
    
    ax(3)=subplot(3,4,3); hold on
    %     plot(TIME,VELx,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[VELxavg'+nSigma*VELxstd';flipud(VELxavg'-nSigma*VELxstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,VELxavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('velocity x_b_o_d_y (m.s^-^1)')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-1.5 3.5])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-2.5 2.5])
    end
    ca=gca;
    set(gca,'XColor','none');
    
    ax(7)=subplot(3,4,7); hold on
    %     plot(TIME,VELy,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[VELyavg'+nSigma*VELystd';flipud(VELyavg'-nSigma*VELystd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,VELyavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('velocity y_b_o_d_y (m.s^-^1)')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-2.5 2.5])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-3.5 1.5])
    end
    ca=gca;
    set(gca,'XColor','none');
    
    ax(11)=subplot(3,4,11); hold on
    %     plot(TIME,VELz,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[VELzavg'+nSigma*VELzstd';flipud(VELzavg'-nSigma*VELzstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,VELzavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('velocity z_b_o_d_y (m.s^-^1)')
    if strcmp(exp_title,'PITCH_STEPS')
        set(gca,'Ylim',[-4.5 0.5])
    elseif strcmp(exp_title,'ROLL_STEPS')
        set(gca,'Ylim',[-4 1])
    end
    xlabel('time (s)')
    
    ax(4)=subplot(3,4,4); hold on
    %     plot(TIME,OM,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[OMavg'+nSigma*OMstd';flipud(OMavg'-nSigma*OMstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,OMavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('rate (s^-^1)')
    set(gca,'Ylim',[-100 500])
    ca=gca;
    ca.YAxis.TickLabelFormat = '%g\\circ';
    set(gca,'XColor','none');
    
    ax(8)=subplot(3,4,8); hold on
    %     plot(TIME,VELOC,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[VELOCavg'+nSigma*VELOCstd';flipud(VELOCavg'-nSigma*VELOCstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,VELOCavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('speed (m.s^-^1)')
    set(gca,'Ylim',[0 5])
    ca=gca;
    set(gca,'XColor','none');
    
    ax(12)=subplot(3,4,12); hold on
    %     plot(TIME,ACCEL,'Color',colors_light(i,:))
    s=fill([TIME';flipud(TIME')],[ACCELavg'+nSigma*ACCELstd';flipud(ACCELavg'-nSigma*ACCELstd')],colors_light(i,:),'linestyle','none');
    alpha(s,.25)
    plot(TIME,ACCELavg,'Color',colors(i,:),'LineWidth',2)
    ylabel('acceleration (m.s^-^2)')
    xlabel('time (s)')
    set(gca,'Ylim',[0 13])
    
end

linkaxes(ax,'x')
set(gca,'Xlim',[-0.1 3])

for i=1:length(NEXP)
    if i==1
        legend_rpy='line_rpy1';
    else
        legend_rpy=[legend_rpy ',line_rpy' num2str(i)];
    end
end

figure(1)
eval(['legend([' legend_rpy '],legend_list)'])
xlim([-0.5 3.5])
% figure(2)
% eval(['legend([' legend_rpy '],legend_list)'])



