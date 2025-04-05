close all
clear all

% !!! If you want to export the movie, set "anim_save" to "on"

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


%% animation with wing motion - Fig2E (exp13_2)

% upsample everything to 120 Hz using interpolation
fps_interp=120;
time_track_interp=time(1):1/fps_interp:time(end);

posCG_E_interp=interp1(time,[posX_aligned;posY_aligned;posZ_aligned]',time_track_interp,'spline');
roll_D2EFF_interp=interp1(time,roll,time_track_interp,'spline')';
pitch_D2EFF_interp=interp1(time,pitch,time_track_interp,'spline')';
yaw_D2EFF_interp=interp1(time,yaw,time_track_interp,'spline')';
FF_track_WG_E_interp=interp1(time,FF_E',time_track_interp,'spline');
velCG_EFF_interp=interp1(time,vel_CG_E_aligned',time_track_interp,'spline');
velCG_DFF_interp=interp1(time,vel_CG_D',time_track_interp,'spline');

% resample commands to OptiTrack time vector
cmd_rollFF_interp=interp1(time_onboard,cmd_roll_filtered,time_track_interp,'spline')';
cmd_pitchFF_interp=interp1(time_onboard,cmd_pitch_filtered,time_track_interp,'spline')';
cmd_yawFF_interp=interp1(time_onboard,cmd_yaw_filtered,time_track_interp,'spline')';
cmd_thrustFF_interp=interp1(time_onboard,cmd_thrust_filtered,time_track_interp,'spline')';
cmd_hingeRFF_interp=interp1(time_onboard,cmd_hingeR_filtered,time_track_interp,'spline')';
cmd_hingeLFF_interp=interp1(time_onboard,cmd_hingeL_filtered,time_track_interp,'spline')';
cmd_motor_leftFF_interp=interp1(time_onboard,cmd_motorL_filtered,time_track_interp,'spline')';
cmd_motor_rightFF_interp=interp1(time_onboard,cmd_motorR_filtered,time_track_interp,'spline')';
freq_interp=interp1(time_freq,freq_right,time_track_interp,'spline')';

% estimate the flapping frequencies

% f = s1*cmd + s2, values according to single wing force balance measurements
s1 = 0.2014;
s2 = 3.9517*0.7; % 70% correction to fit the data

f0=17.1; % estimated flapping frequency at hover

[A,B,C,D]=tf2ss([12.56],[1 12.56]);
sys_motor=ss(A,B,C,D);
fL_cmd=cmd_motor_leftFF_interp*s1+s2-0.5*velCG_DFF_interp(:,3)+0.5*velCG_DFF_interp(:,2); % flapping frequency decrease due to w
fL=lsim(sys_motor,fL_cmd,time_track_interp,f0/C);

fR_cmd=cmd_motor_rightFF_interp*s1+s2-0.5*velCG_DFF_interp(:,3)-0.5*velCG_DFF_interp(:,2); % flapping frequency decrease due to w
fR=lsim(sys_motor,fR_cmd,time_track_interp,f0/C);

% figure
% plot(time_track,fR), hold on
% plot(time_track,freq_interp)
% ylim([0 25])
% return

index=1:length(time_track_interp);
man_start=index(time_track_interp==0); % sample number when time=0
TS=-1.2;
TF=2.8;

anim_start=round(TS*fps_interp)+man_start;
anim_lims=[-1.5 1.2 -2.3 0.4 -0.4 0.3];
        
anim_options={'COG',[0;0;-0.06], ...
              'anim_start',anim_start, ...
              'anim_step',1 ...
              'anim_length',round(TF*fps_interp), ...
              'anim_view','view2', ...
              'anim_traces','on', ...
              'anim_scale',1, ...
              'anim_detail','off', ...
              'anim_delfly','on', ...
              'anim_control','on', ...
              'anim_force','off', ...
              'anim_moment','off', ...
              'anim_speed','off', ...
              'anim_sequence','off', ...
              'anim_align','on', ...
              'anim_zero',round((-TS)*fps_interp), ...
              'anim_lims',anim_lims,...
              'anim_save','off'}; 

          
% animation using commanded dihedral
transformer_animation_wing_motion(posCG_E_interp(:,1),-posCG_E_interp(:,2),-posCG_E_interp(:,3),roll_D2EFF_interp,-pitch_D2EFF_interp,-yaw_D2EFF_interp,cmd_hingeLFF_interp/180*pi,cmd_hingeRFF_interp/180*pi,cmd_motor_leftFF_interp,cmd_motor_rightFF_interp,cmd_rollFF_interp,cmd_pitchFF_interp,cmd_yawFF_interp,cmd_thrustFF_interp,FF_track_WG_E_interp',zeros(size(FF_track_WG_E_interp))',velCG_EFF_interp',fL,fR,time_track_interp,anim_options)
