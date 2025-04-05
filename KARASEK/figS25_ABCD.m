close all
clear all

NEXP=[76 77 78 79];
NMAN=[3 4 2 1];

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

addpath('support_files')
load dataset_revision.mat

colors=viridis(4);
colors_light=colors;


for jj=1:length(NEXP)
    Nexp=NEXP(jj);
    nman=NMAN(jj);
    eval(['data=experiment' num2str(Nexp) ';']);
    
    assign_variables
    
    % this will recolor the wings
    cmd_motorL_interp=33.33*(jj-1)*ones(size(time));
    cmd_motorR_interp=33.33*(jj-1)*ones(size(time));
    
    % Body forces - ground fixed frame
    XX=m*acc_CG_E_aligned_x;
    YY=m*acc_CG_E_aligned_y;
    ZZ=m*acc_CG_E_aligned_z;
    
    FF_E=[XX; YY; ZZ];
    
    %% Sequence - Science Fig. 4
    index=1:length(time);
    man_start=index(time==0); % sample number when time=0
    
    TS=0;
    TF=0.8;
    fps=1/median(diff(time));
    
    anim_start=round((TS)*fps)+man_start;
    anim_lims=[-0.2 0.6 -1 0.25 -2 2];
    step=round(fps*TF);
    
    anim_options={'COG',[0;0;-0.06], ...
        'anim_start',anim_start, ...
        'anim_step',step, ...
        'anim_length',fps*TF, ...
        'anim_view','top', ...
        'anim_traces','on', ...
        'anim_scale',1, ...
        'anim_detail','off', ...
        'anim_delfly','on', ...
        'anim_control','on', ...
        'anim_force','on', ...
        'anim_force_scale',0.2, ...
        'anim_moment','off', ...
        'anim_speed','on', ...
        'anim_speed_scale',0.1, ...
        'anim_sequence','on', ...
        'anim_align','off', ...
        'anim_zero',0, ...
        'anim_lims',anim_lims};
    
    transformer_animation_evasion(posX_aligned,-posY_aligned,-posZ_aligned,roll,-pitch,-yaw_aligned,cmd_hingeL_interp,cmd_hingeR_interp,cmd_motorL_interp,cmd_motorR_interp,cmd_yaw_interp,FF_E,zeros(size(FF_E)),vel_CG_E_aligned,time,m,anim_options)
    
    assign_repetition_variables
    
    TIME1=TIME(TIME<=TF);
    select1=zeros(size(TIME));
    select2=zeros(size(TIME));
    select1(TIME<=TF)=1;
    select2(TIME>=TS)=1;
    select=select1.*select2;
    
    hold on
    plot3(POSx(:,select==1)',-POSy(:,select==1)',-POSz(:,select==1)','Color',colors_light(jj,:))
    eval(['line_pos' num2str(jj) '=plot3(POSxavg(:,select==1),-POSyavg(:,select==1),-POSzavg(:,select==1),''Color'',colors(jj,:),''LineWidth'',2);'])
    
end