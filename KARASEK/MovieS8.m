close all
clear all

% !!! If you want to export the movie, set "anim_save" to "on"

Nexp=56;
nman=3;

m=29.85e-3; % complete setup, with markers, standing gear and a battery
g=9.81;

addpath('support_files')
load dataset_revision.mat
eval(['data=experiment' num2str(Nexp) ';']);

assign_variables

% comment the following 2 lines if motor commands should be represented by the wing color
cmd_motorL_interp=50*ones(size(time));
cmd_motorR_interp=50*ones(size(time));

% Body forces - ground fixed frame
% XX=m*acc_CG_E_x;
% YY=m*acc_CG_E_y;
% ZZ=m*acc_CG_E_z;
XX=m*acc_CG_E_aligned_x;
YY=m*acc_CG_E_aligned_y;
ZZ=m*acc_CG_E_aligned_z; 

FF_E=[XX; YY; ZZ];

% estimate the flapping frequencies

% f = s1*cmd + s2, values according to single wing force balance measurements
s1 = 0.2014;
s2 = 3.9517*0.8; % 90% correction to fit the data

f0=17.1; % estimated flapping frequency at hover

[A,B,C,D]=tf2ss([12.56],[1 12.56]);
sys_motor=ss(A,B,C,D);
fL_cmd=cmd_motorL_interp*s1+s2-0.5*vel_CG_D(3,:)+0.5*vel_CG_D(2,:); % flapping frequency decrease due to w
fL=lsim(sys_motor,fL_cmd,time,f0/C)';

fR_cmd=cmd_motorR_interp*s1+s2-0.5*vel_CG_D(3,:)-0.5*vel_CG_D(2,:); % flapping frequency decrease due to w
fR=lsim(sys_motor,fR_cmd,time,f0/C)';

% figure
% plot(time_track,fR), hold on
% plot(time_track,freq_interp)
% ylim([0 25])
% return

index=1:length(time);
man_start=index(time==0); % sample number when time=0
TS=-1.2;
TF=2.8;

fps=1/median(diff(time));

anim_start=round((TS)*fps)+man_start;
anim_lims=[-1.2 0.8 -1 0.4 -0.4 0.3];
        
anim_options={'COG',[0;0;-0.06], ...
              'anim_start',anim_start, ...
              'anim_step',1 ...
              'anim_length',round(TF*fps), ...
              'anim_view','top', ...
              'anim_traces','off', ...
              'anim_scale',1, ...
              'anim_detail','off', ...
              'anim_delfly','on', ...
              'anim_control','on', ...
              'anim_force','off', ...
              'anim_moment','off', ...
              'anim_speed','off', ...
              'anim_sequence','off', ...
              'anim_align','on', ...
              'anim_zero',round((-TS)*fps), ...
              'anim_lims',anim_lims,...
              'anim_save','off'}; 

transformer_animation_wing_motion_grey(posX_aligned,-posY_aligned,-posZ_aligned,roll,-pitch,-yaw_aligned,cmd_hingeL_interp,cmd_hingeR_interp,cmd_motorL_interp,cmd_motorR_interp,cmd_roll_interp,cmd_pitch_interp,cmd_yaw_interp,cmd_thrust_interp,FF_E,zeros(size(FF_E)),vel_CG_E_aligned,fL,fR,time,anim_options)
