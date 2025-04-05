close all
clear all

Nexp=13;
nman=1; % I only saved one maneuver in the Science dataset, it is number 2 in the original data

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables

% Body forces
% XX=m*acc_CG_E_x;
% YY=m*acc_CG_E_y;
% ZZ=m*acc_CG_E_z; 
XX=m*acc_CG_E_aligned_x;
YY=m*acc_CG_E_aligned_y;
ZZ=m*acc_CG_E_aligned_z; 

FF_E=[XX; YY; ZZ];

%% Sequence

index=1:length(time);
man_start=index(time==0); % sample number when time=0
TS=-0.125;
TF=1.125;

fps=1/median(diff(time));

anim_start=round(TS*fps)+man_start;
anim_lims=[-0.35 1.2 -1.7 0.4 -0.4 0.3];

step=round(fps*0.125);

anim_options={'COG',[0;0;-0.06], ...
    'anim_start',anim_start, ...
    'anim_step',step, ...
    'anim_length',fps*TF, ...
    'anim_view','iso', ...
    'anim_traces','on', ...
    'anim_scale',1, ...
    'anim_detail','off', ...
    'anim_delfly','on', ...
    'anim_control','on', ...
    'anim_force','on', ...
    'anim_force_scale',0.1,...
    'anim_moment','off', ...
    'anim_speed','on', ...
    'anim_sequence','on', ...
    'anim_align','off', ...
    'anim_zero',step, ...
    'anim_lims',anim_lims};

% transformer_animation_evasion(posX,-posY,-posZ,roll,-pitch,-yaw,cmd_hingeL_interp,cmd_hingeR_interp,cmd_motorL_interp,cmd_motorR_interp,cmd_yaw_interp,FF_E,zeros(size(FF_E)),vel_CG_E,time,m,anim_options)
transformer_animation_evasion(posX_aligned,-posY_aligned,-posZ_aligned,roll,-pitch,-yaw_aligned,cmd_hingeL_interp,cmd_hingeR_interp,cmd_motorL_interp,cmd_motorR_interp,cmd_yaw_interp,FF_E,zeros(size(FF_E)),vel_CG_E_aligned,time,m,anim_options)
