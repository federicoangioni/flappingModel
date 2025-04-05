close all
clear all
clc

addpath('support_files')
addpath('fly_data')

% fruit fly dataset from Muijres 2014 Science Supplement
load FRUITFLY_LOOMINGRESPONSE_DATABASE.mat

load dataset_revision.mat

% data for fig. 2AB
Nexp_fly=11; % fig. 2B
data=experiment56;% fig. 2A
Nexp=3;
turn_dir=-1;

assign_repetition_variables

%% scaling factors
g=9.81;

% fruit fly
f_fly=188.7;
R_fly=wing_data.wing_length_LnR(Nexp_fly)/1000;
m_fly=settings_variables.Mfly;
stroke_plane_ang=settings_variables.strokeplane_angle/180*pi;
AR_fly=settings_variables.ARwing_fly;
c_fly=R_fly/AR_fly;
S_fly=R_fly^2/AR_fly;

joint_posL_sp=Rot_y(-stroke_plane_ang)*wing_data.joint_pos_L(:,Nexp_fly)/1000;
joint_posR_sp=Rot_y(-stroke_plane_ang)*wing_data.joint_pos_R(:,Nexp_fly)/1000;

% Muijres et al. 2015 Body saccades of Drosophila consist of stereotyped banked turns"
Ixx_fly=0.64*m_fly*g*R_fly/f_fly^2;
Iyy_fly=1.07*m_fly*g*R_fly/f_fly^2;
Izz_fly=0.57*m_fly*g*R_fly/f_fly^2;

Croll_fly=0.22*m_fly*g*R_fly/f_fly;
Cpitch_fly=0.08*m_fly*g*R_fly/f_fly;
Cyaw_fly=0.41*m_fly*g*R_fly/f_fly;

% Transformer
f_trans=2*17;
% f_trans=2.1*17
R_trans=0.14;

% scaling
length_scale=R_trans/R_fly;
time_scale=f_trans/f_fly; % flapping frequency ratio

%% fly data
% position and velocity
posx_fly=body_data.pos(:,Nexp_fly,1);
posy_fly=-body_data.pos(:,Nexp_fly,2); % sign changed to use aerospace convention
posz_fly=-body_data.pos(:,Nexp_fly,3); % sign changed to use aerospace convention

velx_fly=body_data.vel(:,Nexp_fly,1);
vely_fly=-body_data.vel(:,Nexp_fly,2); % sign changed to use aerospace convention
velz_fly=-body_data.vel(:,Nexp_fly,3); % sign changed to use aerospace convention

% rates are already using aerospace convention
omx_fly_body=body_data.omega(:,Nexp_fly,1);
omy_fly_body=body_data.omega(:,Nexp_fly,2);
omz_fly_body=body_data.omega(:,Nexp_fly,3);

% quaternions - reference axes rotated 180deg around x compared to aerospace convention
q0=body_data.qbody(:,Nexp_fly,4);
q1=body_data.qbody(:,Nexp_fly,1);
q2=body_data.qbody(:,Nexp_fly,2);
q3=body_data.qbody(:,Nexp_fly,3);

% Euler angles (body) - reference axes rotated 180deg around x compared to aerospace convention
roll_fly_body0=atan2(2*(q0.*q1+q2.*q3),1-2*(q1.^2+q2.^2));
pitch_fly_body0=asin(2*(q0.*q2-q3.*q1));
yaw_fly_body0=atan2(2*(q0.*q3+q1.*q2),1-2*(q2.^2+q3.^2));

time_fly=body_data.t;

Nsamples=length(time_fly);
index=1:Nsamples;

% keep only tracked samples
posx_fly_tracked=posx_fly(~isnan(posx_fly));
posy_fly_tracked=posy_fly(~isnan(posy_fly));
posz_fly_tracked=posz_fly(~isnan(posz_fly));

time0_fly=time_fly(~isnan(posx_fly));
index_tracked=index(~isnan(posx_fly));

dt=time0_fly(2)-time0_fly(1);

% reset position and heading at the beginning, mirror axes to change a left
% hand turn to right hand one if needed (turn_dir)
NN=20;
heading0=atan2(posy_fly_tracked(NN)-posy_fly_tracked(1),posx_fly_tracked(NN)-posx_fly_tracked(1));

posx_fly_aligned=(posx_fly-posx_fly_tracked(1))*cos(heading0)+(posy_fly-posy_fly_tracked(1))*sin(heading0);
posy_fly_aligned=(-(posx_fly-posx_fly_tracked(1))*sin(heading0)+(posy_fly-posy_fly_tracked(1))*cos(heading0));
posz_fly_aligned=-(posz_fly-posz_fly_tracked(1));

course_fly=atan2(vely_fly,velx_fly)-heading0;
course_fly=make_angle_continuous(course_fly);

% figure
% plot(posx_fly_aligned,posy_fly_aligned), axis equal, hold on
% plot(posx_fly_aligned,posy_fly_aligned)
% plot3(posx_fly,posy_fly,posz_fly)

% transform rates to stroke plane reference frame
omx_fly_sp=omx_fly_body*cos(stroke_plane_ang)-omz_fly_body*sin(stroke_plane_ang);
omy_fly_sp=omy_fly_body;
omz_fly_sp=omx_fly_body*sin(stroke_plane_ang)+omz_fly_body*cos(stroke_plane_ang);

% scaling
posx_fly_scaled=posx_fly_aligned/R_fly;
posy_fly_scaled=posy_fly_aligned/R_fly;
posz_fly_scaled=posz_fly_aligned/R_fly;

% scale time to wingbeats
t_fly_scaled=time_fly*f_fly;

% start and end of the fly maneuver
tw_min=time0_fly(1)*f_fly;
tw_max=time0_fly(end)*f_fly;

% scale rates to deg/wingbeat
omx_fly_sp_scaled=omx_fly_sp/pi*180/f_fly;
omy_fly_sp_scaled=omy_fly_sp/pi*180/f_fly;
omz_fly_sp_scaled=omz_fly_sp/pi*180/f_fly;

roll_est_sp=NaN(size(omx_fly_body));
pitch_est_sp=NaN(size(omx_fly_body));
yaw_est_sp=NaN(size(omx_fly_body));

roll_est_body=NaN(size(omx_fly_body));
pitch_est_body=NaN(size(omx_fly_body));
yaw_est_body=NaN(size(omx_fly_body));

% figure, hold on % plots trajectory with coordinate frames

for i=1:Nsamples
    % Wikipedia
    % Rbody=[1-2*(q2(i)^2+q3(i)^2)       , 2*(q1(i)*q2(i)-q0(i)*q3(i)), 2*(q0(i)*q2(i)+q1(i)*q3(i));
    %        2*(q1(i)*q2(i)+q0(i)*q3(i)) , 1-2*(q1(i)^2+q3(i)^2)      , 2*(q2(i)*q3(i)-q0(i)*q1(i));
    %        2*(q1(i)*q3(i)-q0(i)*q2(i)) , 2*(q0(i)*q1(i)+q2(i)*q3(i)), 1-2*(q1(i)^2+q2(i)^2)      ];
    %   
    % formula from Fontaine 2009
    Rbody=[q0(i)^2+q1(i)^2-q2(i)^2-q3(i)^2    , 2*(q1(i)*q2(i)-q0(i)*q3(i)), 2*(q0(i)*q2(i)+q1(i)*q3(i));
           2*(q1(i)*q2(i)+q0(i)*q3(i)) , q0(i)^2-q1(i)^2+q2(i)^2-q3(i)^2      , 2*(q2(i)*q3(i)-q0(i)*q1(i));
           2*(q1(i)*q3(i)-q0(i)*q2(i)) , 2*(q0(i)*q1(i)+q2(i)*q3(i)), q0(i)^2-q1(i)^2-q2(i)^2+q3(i)^2      ];
    
    RbodyInv=[q0(i)^2+q1(i)^2-q2(i)^2-q3(i)^2 , 2*(q1(i)*q2(i)+q0(i)*q3(i))     , 2*(q1(i)*q3(i)-q0(i)*q2(i));
              2*(q1(i)*q2(i)-q0(i)*q3(i))     , q0(i)^2-q1(i)^2+q2(i)^2-q3(i)^2 , 2*(q0(i)*q1(i)+q2(i)*q3(i));
              2*(q0(i)*q2(i)+q1(i)*q3(i))     , 2*(q2(i)*q3(i)-q0(i)*q1(i))     , q0(i)^2-q1(i)^2-q2(i)^2+q3(i)^2 ];
          
    Rbody=Rot_x(pi)*Rbody; % rotate 180deg around y to comply with aerospace convention
    RbodyInv=RbodyInv*Rot_x(pi); % rotate 180deg around y to comply with aerospace convention
    
    % compute Euler's in body frame (Aerospace convention)
    roll_fly_body(i)=atan2(Rbody(3,2),Rbody(3,3));
    pitch_fly_body(i)=atan2(-Rbody(3,1),real(sqrt(1-Rbody(3,1)^2))); % real added to avoid complex numbers (most likely due to rounding errors)
    yaw_fly_body(i)=atan2(Rbody(2,1),Rbody(1,1));
    
    Rsp=Rbody*Rot_y(stroke_plane_ang);
    RspInv=Rot_y(-stroke_plane_ang)*RbodyInv;
    
    Rot_sp(:,:,i)=Rsp;
        
    roll_fly_sp(i,1)=atan2(Rsp(3,2),Rsp(3,3));
    pitch_fly_sp(i,1)=atan2(-Rsp(3,1),real(sqrt(1-Rsp(3,1)^2))); % real added to avoid complex numbers (most likely due to rounding errors)
    yaw_fly_sp(i,1)=atan2(Rsp(2,1),Rsp(1,1));
    
    % Using zxy order: Rzxy=Rot_z(psi)*Rot_x(phi)*Rot_y(theta)
    roll_zxy_sp(i,1)=atan2(Rsp(3,2),real(sqrt(1-Rsp(3,2)^2)));
    pitch_zxy_sp(i,1)=atan2(-Rsp(3,1),Rsp(3,3)); % real added to avoid complex numbers (most likely due to rounding errors)
    yaw_zxy_sp(i,1)=atan2(-Rsp(1,2),Rsp(2,2));
    
    vbody=RbodyInv*[velx_fly(i);vely_fly(i);velz_fly(i)];
    velx_fly_body(i,1)=vbody(1);
    vely_fly_body(i,1)=vbody(2);
    velz_fly_body(i,1)=vbody(3);
    
    vsp=RspInv*[velx_fly(i);vely_fly(i);velz_fly(i)];
    velx_fly_sp(i,1)=vsp(1);
    vely_fly_sp(i,1)=vsp(2);
    velz_fly_sp(i,1)=vsp(3);
    
    % integrate rates to estimate attitude
    if i==index_tracked(1)
        roll_est_sp(i,1)=roll_fly_sp(i);
        pitch_est_sp(i,1)=pitch_fly_sp(i);
        yaw_est_sp(i,1)=yaw_fly_sp(i);
        
        roll_est_body(i,1)=roll_fly_body(i);
        pitch_est_body(i,1)=pitch_fly_body(i);
        yaw_est_body(i,1)=yaw_fly_body(i);
    end
    if i>index_tracked(1)
        roll_est_sp(i,1)=roll_est_sp(i-1)+omx_fly_sp(i-1)*dt+sin(roll_est_sp(i-1))*tan(pitch_est_sp(i-1))*omy_fly_sp(i-1)*dt+cos(roll_est_sp(i-1))*tan(pitch_est_sp(i-1))*omz_fly_sp(i-1)*dt;
        pitch_est_sp(i,1)=pitch_est_sp(i-1)+cos(roll_est_sp(i-1))*omy_fly_sp(i-1)*dt-sin(roll_est_sp(i-1))*omz_fly_sp(i-1)*dt;
        yaw_est_sp(i,1)=yaw_est_sp(i-1)+sin(roll_est_sp(i-1))/cos(pitch_est_sp(i-1))*omy_fly_sp(i-1)*dt+cos(roll_est_sp(i-1))/cos(pitch_est_sp(i-1))*omz_fly_sp(i-1)*dt;
        
        roll_est_body(i,1)=roll_est_body(i-1)+omx_fly_body(i-1)*dt+sin(roll_est_body(i-1))*tan(pitch_est_body(i-1))*omy_fly_body(i-1)*dt+cos(roll_est_body(i-1))*tan(pitch_est_body(i-1))*omz_fly_body(i-1)*dt;
        pitch_est_body(i,1)=pitch_est_body(i-1)+cos(roll_est_body(i-1))*omy_fly_body(i-1)*dt-sin(roll_est_body(i-1))*omz_fly_body(i-1)*dt;
        yaw_est_body(i,1)=yaw_est_body(i-1)+sin(roll_est_body(i-1))/cos(pitch_est_body(i-1))*omy_fly_body(i-1)*dt+cos(roll_est_body(i-1))/cos(pitch_est_body(i-1))*omz_fly_body(i-1)*dt;
    end
    
end

yaw_fly_sp=yaw_fly_sp-heading0;
yaw_fly_sp=make_angle_continuous(yaw_fly_sp);


%% Transformer data
t_trans_scaled=TIME*f_trans;

omx_trans_scaled=OMx(Nexp,:)/f_trans;
omy_trans_scaled=OMy(Nexp,:)/f_trans;
omz_trans_scaled=OMz(Nexp,:)/f_trans;

velx_trans_scaled=VELx(Nexp,:)/R_trans/f_trans;
vely_trans_scaled=VELy(Nexp,:)/R_trans/f_trans;
velz_trans_scaled=VELz(Nexp,:)/R_trans/f_trans;
VEL_trans_scaled=VELOC(Nexp,:)/R_trans/f_trans;
bet_trans=BET(Nexp,:);
course_trans=HEADaligned(Nexp,:);

roll_trans=ROLL(Nexp,:);
pitch_trans=PITCH(Nexp,:);
yaw_trans=YAWaligned(Nexp,:);

%% transform to forward flight axes

pitch0=-35/180*pi;

% transform rates to stroke plane reference frame
omx_trans_scaled_sp=omx_trans_scaled*cos(pitch0)-omz_trans_scaled*sin(pitch0);
omy_trans_scaled_sp=omy_trans_scaled;
omz_trans_scaled_sp=omx_trans_scaled*sin(pitch0)+omz_trans_scaled*cos(pitch0);

%% interpolate and do cross-corellation to find the time delay
ind_fly_tracked=1:length(omx_fly_sp_scaled);
ind_fly_tracked=ind_fly_tracked(~isnan(omx_fly_sp_scaled));

t_fly_scaled1=t_fly_scaled(ind_fly_tracked);
omx_fly_sp_scaled1=omx_fly_sp_scaled(ind_fly_tracked);
omy_fly_sp_scaled1=omy_fly_sp_scaled(ind_fly_tracked);
omz_fly_sp_scaled1=omz_fly_sp_scaled(ind_fly_tracked);

dt_int=0.01;
t_trans_int=t_trans_scaled(1):dt_int:t_trans_scaled(end);
omx_trans_int=interp1(t_trans_scaled,omx_trans_scaled,t_trans_int,'spline');
omy_trans_int=interp1(t_trans_scaled,omy_trans_scaled,t_trans_int,'spline');
omz_trans_int=interp1(t_trans_scaled,omz_trans_scaled,t_trans_int,'spline');

t_fly_int=tw_min:dt_int:tw_max;
omx_fly_int=interp1(t_fly_scaled1,omx_fly_sp_scaled1,t_fly_int,'spline');
omy_fly_int=interp1(t_fly_scaled1,omy_fly_sp_scaled1,t_fly_int,'spline');
omz_fly_int=interp1(t_fly_scaled1,omz_fly_sp_scaled1,t_fly_int,'spline');

% crosscorellation to find the delay
[omx_acor,omx_lag]=xcorr(turn_dir*omx_fly_int,omx_trans_int);
[~,iomx]=max(omx_acor);
omx_delay=omx_lag(iomx)*dt_int;

[omy_acor,omy_lag]=xcorr(omy_fly_int,omy_trans_int);
[~,iomy]=max(omy_acor);
omy_delay=omy_lag(iomy)*dt_int;

% figure,plot(omx_lag,omx_acor),hold on,plot(omy_lag,omy_acor)

delay=(omx_delay+omy_delay)/2;

% align the time vectors
t_trans_scaled_aligned=t_trans_scaled-t_trans_scaled(1)+delay;
t_fly_scaled_aligned=t_fly_scaled-t_fly_scaled1(1);

t_trans_int1=t_trans_int-t_trans_int(1)+delay;
t_fly_int1=t_fly_int-t_fly_int(1);

% figure
% ax_om(1)=subplot(2,1,1);
% plot(t_fly_int1,turn_dir*omx_fly_int),hold on
% plot(t_trans_int,omx_trans_int)
% plot(t_trans_int1,omx_trans_int)
% legend('fly','transformer original','transformer shifted')
% 
% ax_om(2)=subplot(2,1,2);
% plot(t_fly_int1,omy_fly_int),hold on
% plot(t_trans_int,omy_trans_int)
% plot(t_trans_int1,omy_trans_int)
% xlabel('time')
% 
% linkaxes(ax_om,'x')

%% reset Transformer position
ind_reset=1:length(POSx(Nexp,:));
ind_reset=ind_reset(t_trans_scaled_aligned>=0);
ind_reset=ind_reset(1);

posx_trans_scaled=(POSx(Nexp,:)-POSx(Nexp,ind_reset))/R_trans;
posy_trans_scaled=(POSy(Nexp,:)-POSy(Nexp,ind_reset))/R_trans;
posz_trans_scaled=(POSz(Nexp,:)-POSz(Nexp,ind_reset))/R_trans;

%% compute correlation coefficients
ind_trans_int=1:length(t_trans_int1);
ind_cropped=ind_trans_int(t_trans_int1>=0);
ind_cropped=ind_cropped(1:length(t_fly_int1));

t_trans_int_cropped=t_trans_int1(ind_cropped);

omx_trans_int_cropped=omx_trans_int(ind_cropped);
omy_trans_int_cropped=omy_trans_int(ind_cropped);
omz_trans_int_cropped=omz_trans_int(ind_cropped);

% interpolate the remaining variables
posx_trans_int=interp1(t_trans_scaled,posx_trans_scaled,t_trans_int,'spline');
posy_trans_int=interp1(t_trans_scaled,posy_trans_scaled,t_trans_int,'spline');
posz_trans_int=interp1(t_trans_scaled,posz_trans_scaled,t_trans_int,'spline');

velx_trans_int=interp1(t_trans_scaled,velx_trans_scaled,t_trans_int,'spline');
vely_trans_int=interp1(t_trans_scaled,vely_trans_scaled,t_trans_int,'spline');
velz_trans_int=interp1(t_trans_scaled,velz_trans_scaled,t_trans_int,'spline');
VEL_trans_int=interp1(t_trans_scaled,VEL_trans_scaled,t_trans_int,'spline');
bet_trans_int=interp1(t_trans_scaled,bet_trans,t_trans_int,'spline');
course_trans_int=interp1(t_trans_scaled,course_trans,t_trans_int,'spline');

roll_trans_int=interp1(t_trans_scaled,roll_trans,t_trans_int,'spline');
pitch_trans_int=interp1(t_trans_scaled,pitch_trans,t_trans_int,'spline');
yaw_trans_int=interp1(t_trans_scaled,yaw_trans,t_trans_int,'spline');

posx_trans_int_cropped=posx_trans_int(ind_cropped);
posy_trans_int_cropped=posy_trans_int(ind_cropped);
posz_trans_int_cropped=posz_trans_int(ind_cropped);

velx_trans_int_cropped=velx_trans_int(ind_cropped);
vely_trans_int_cropped=vely_trans_int(ind_cropped);
velz_trans_int_cropped=velz_trans_int(ind_cropped);
VEL_trans_int_cropped=VEL_trans_int(ind_cropped);
bet_trans_int_cropped=bet_trans_int(ind_cropped);
course_trans_int_cropped=course_trans_int(ind_cropped);

roll_trans_int_cropped=roll_trans_int(ind_cropped);
pitch_trans_int_cropped=pitch_trans_int(ind_cropped);
yaw_trans_int_cropped=yaw_trans_int(ind_cropped);


posx_fly_int=interp1(t_fly_scaled1,posx_fly_scaled(ind_fly_tracked),t_fly_int,'spline');
posy_fly_int=interp1(t_fly_scaled1,posy_fly_scaled(ind_fly_tracked),t_fly_int,'spline');
posz_fly_int=interp1(t_fly_scaled1,posz_fly_scaled(ind_fly_tracked),t_fly_int,'spline');

velx_fly_scaled=velx_fly_sp/R_fly/f_fly;
vely_fly_scaled=vely_fly_sp/R_fly/f_fly;
velz_fly_scaled=velz_fly_sp/R_fly/f_fly;

VEL_fly_scaled=(velx_fly_scaled.^2+vely_fly_scaled.^2+velz_fly_scaled.^2).^0.5;
bet_fly=asind(vely_fly_scaled./VEL_fly_scaled);

velx_fly_int=interp1(t_fly_scaled1,velx_fly_scaled(ind_fly_tracked),t_fly_int,'spline');
vely_fly_int=interp1(t_fly_scaled1,vely_fly_scaled(ind_fly_tracked),t_fly_int,'spline');
velz_fly_int=interp1(t_fly_scaled1,velz_fly_scaled(ind_fly_tracked),t_fly_int,'spline');

VEL_fly_int=interp1(t_fly_scaled1,VEL_fly_scaled(ind_fly_tracked),t_fly_int,'spline');
bet_fly_int=interp1(t_fly_scaled1,bet_fly(ind_fly_tracked),t_fly_int,'spline');
course_fly_int=interp1(t_fly_scaled1,course_fly(ind_fly_tracked)/pi*180,t_fly_int,'spline');

roll_fly_int=interp1(t_fly_scaled1,roll_fly_sp(ind_fly_tracked)/pi*180,t_fly_int,'spline');
pitch_fly_int=interp1(t_fly_scaled1,pitch_fly_sp(ind_fly_tracked)/pi*180,t_fly_int,'spline');
yaw_fly_int=interp1(t_fly_scaled1,yaw_fly_sp(ind_fly_tracked)/pi*180,t_fly_int,'spline');

% figure
% subplot(3,1,1)
% plot(omx_fly_int), hold on, plot(omx_trans_int_cropped)
% 
% subplot(3,1,2)
% plot(omy_fly_int), hold on, plot(omy_trans_int_cropped)
% 
% subplot(3,1,3)
% plot(omz_fly_int), hold on, plot(omz_trans_int_cropped)

[corrcoef_omx P_omx]=corrcoef(turn_dir*omx_fly_int,omx_trans_int_cropped);
[corrcoef_omy P_omy]=corrcoef(omy_fly_int,omy_trans_int_cropped);
[corrcoef_omz P_omz]=corrcoef(turn_dir*omz_fly_int,omz_trans_int_cropped);

[corrcoef_posx P_posx]=corrcoef(posx_fly_int,posx_trans_int_cropped);
[corrcoef_posy P_posy]=corrcoef(turn_dir*posy_fly_int,posy_trans_int_cropped);
[corrcoef_posz P_posz]=corrcoef(posz_fly_int,posz_trans_int_cropped);

[corrcoef_velx P_velx]=corrcoef(velx_fly_int,velx_trans_int_cropped);
[corrcoef_vely P_vely]=corrcoef(turn_dir*vely_fly_int,vely_trans_int_cropped);
[corrcoef_velz P_velz]=corrcoef(velz_fly_int,velz_trans_int_cropped);

[corrcoef_VEL P_VEL]=corrcoef(VEL_fly_int,VEL_trans_int_cropped);
[corrcoef_bet P_bet]=corrcoef(turn_dir*bet_fly_int,bet_trans_int_cropped);
[corrcoef_course P_course]=corrcoef(turn_dir*course_fly_int,course_trans_int_cropped);

[corrcoef_roll P_roll]=corrcoef(turn_dir*roll_fly_int,roll_trans_int_cropped);
[corrcoef_pitch P_pitch]=corrcoef(pitch_fly_int,pitch_trans_int_cropped);
[corrcoef_yaw P_yaw]=corrcoef(turn_dir*(yaw_fly_int),yaw_trans_int_cropped);

omx_err=turn_dir*omx_fly_int-omx_trans_int_cropped;
omx_rmse=sqrt(mean(omx_err.^2));
omx_rmse_norm=sqrt(mean(omx_err.^2))/max(abs(omx_trans_int_cropped));
omy_err=omy_fly_int-omy_trans_int_cropped;
omy_rmse=sqrt(mean(omy_err.^2));
omy_rmse_norm=sqrt(mean(omy_err.^2))/max(abs(omy_trans_int_cropped));
omz_err=turn_dir*omz_fly_int-omz_trans_int_cropped;
omz_rmse=sqrt(mean(omz_err.^2));
omz_rmse_norm=sqrt(mean(omz_err.^2))/max(abs(omz_trans_int_cropped));
velx_err=velx_fly_int-velx_trans_int_cropped;
velx_rmse=sqrt(mean(velx_err.^2));
velx_rmse_norm=sqrt(mean(velx_err.^2))/max(abs(velx_trans_int_cropped));
vely_err=turn_dir*vely_fly_int-vely_trans_int_cropped;
vely_rmse=sqrt(mean(vely_err.^2));
vely_rmse_norm=sqrt(mean(vely_err.^2))/max(abs(vely_trans_int_cropped));
velz_err=velz_fly_int-velz_trans_int_cropped;
velz_rmse=sqrt(mean(velz_err.^2));
velz_rmse_norm=sqrt(mean(velz_err.^2))/max(abs(velz_trans_int_cropped));
VEL_err=VEL_fly_int-VEL_trans_int_cropped;
VEL_rmse=sqrt(mean(VEL_err.^2));
VEL_rmse_norm=sqrt(mean(VEL_err.^2))/max(abs(VEL_trans_int_cropped));
bet_err=turn_dir*bet_fly_int-bet_trans_int_cropped;
bet_rmse=sqrt(mean(bet_err.^2));
bet_rmse_norm=sqrt(mean(bet_err.^2))/max(abs(bet_trans_int_cropped));
course_err=turn_dir*course_fly_int-course_trans_int_cropped;
course_rmse=sqrt(mean(course_err.^2));
course_rmse_norm=sqrt(mean(course_err.^2))/max(abs(course_trans_int_cropped));
roll_err=turn_dir*roll_fly_int-roll_trans_int_cropped;
roll_rmse=sqrt(mean(roll_err.^2));
roll_rmse_norm=sqrt(mean(roll_err.^2))/max(abs(roll_trans_int_cropped));
pitch_err=pitch_fly_int-pitch_trans_int_cropped;
pitch_rmse=sqrt(mean(pitch_err.^2));
pitch_rmse_norm=sqrt(mean(pitch_err.^2))/max(abs(pitch_trans_int_cropped));
yaw_err=turn_dir*yaw_fly_int-yaw_trans_int_cropped;
yaw_rmse=sqrt(mean(yaw_err.^2));
yaw_rmse_norm=sqrt(mean(yaw_err.^2))/max(abs(yaw_trans_int_cropped));
posx_err=posx_fly_int-posx_trans_int_cropped;
posx_rmse=sqrt(mean(posx_err.^2));
posx_rmse_norm=sqrt(mean(posx_err.^2))/max(abs(posx_trans_int_cropped));
posy_err=turn_dir*posy_fly_int-posy_trans_int_cropped;
posy_rmse=sqrt(mean(posy_err.^2));
posy_rmse_norm=sqrt(mean(posy_err.^2))/max(abs(posy_trans_int_cropped));
posz_err=posz_fly_int-posz_trans_int_cropped;
posz_rmse=sqrt(mean(posz_err.^2));
posz_rmse_norm=sqrt(mean(posz_err.^2))/max(abs(posz_trans_int_cropped));


% figure
% plot(posx_fly_int), hold on
% plot(posx_trans_int_cropped)
% legend('fly','robot')
% 
% figure
% plot(omx_fly_int), hold on
% plot(omx_trans_int_cropped)
% legend('fly','robot')
% 
% figure
% subplot(3,1,1)
% plot(roll_fly_int), hold on
% plot(roll_trans_int_cropped)
% 
% subplot(3,1,2)
% plot(pitch_fly_int), hold on
% plot(pitch_trans_int_cropped)
% 
% subplot(3,1,3)
% plot(yaw_fly_int), hold on
% plot(yaw_trans_int_cropped)

%% robot vs fruit fly comparison
close all

figure('Position',[100 100 800 360])

subplot(3,3,1)
plot(t_fly_scaled_aligned,turn_dir*omx_fly_sp_scaled), hold on
plot(t_trans_scaled_aligned,omx_trans_scaled)
% plot(t_trans_scaled_aligned,omx_trans_scaled_sp)
% plot(t_fly_scaled_aligned,turn_dir*omx_fly_body/f_fly/pi*180), hold on
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('p ((half)wingbeat^-^1)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*omx_fly_sp_scaled),['Corr. coeff. ' num2str(corrcoef_omx(2,1))])
% legend('fly SP','trans body','trans forw.','fly body');
box off
set(gca,'XColor','none');

subplot(3,3,4)
plot(t_fly_scaled_aligned,omy_fly_sp_scaled), hold on
plot(t_trans_scaled_aligned,omy_trans_scaled)
% plot(t_trans_scaled_aligned,omy_trans_scaled_sp)
% plot(t_fly_scaled_aligned,omy_fly_body/f_fly/pi*180), hold on
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('q ((half)wingbeat^-^1)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(omy_fly_sp_scaled),['Corr. coeff. ' num2str(corrcoef_omy(2,1))])
box off
set(gca,'XColor','none');

subplot(3,3,7)
plot(t_fly_scaled_aligned,turn_dir*omz_fly_sp_scaled), hold on
plot(t_trans_scaled_aligned,omz_trans_scaled)
% plot(t_trans_scaled_aligned,omz_trans_scaled_sp)
% plot(t_fly_scaled_aligned,turn_dir*omz_fly_body/f_fly/pi*180), hold on
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('q ((half)wingbeat^-^1)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*omz_fly_sp_scaled),['Corr. coeff. ' num2str(corrcoef_omz(2,1))])
box off

xlabel('(half)wingbeats')
legend('fly','robot','location','northwest')


% figure
subplot(3,3,2)
plot(t_fly_scaled_aligned,turn_dir*roll_fly_sp/pi*180), hold on
plot(t_trans_scaled_aligned,roll_trans)
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('roll')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*roll_fly_int),['Corr. coeff. ' num2str(corrcoef_roll(2,1))])
box off
set(gca,'XColor','none');

subplot(3,3,5)
plot(t_fly_scaled_aligned,pitch_fly_sp/pi*180), hold on
plot(t_trans_scaled_aligned,pitch_trans)
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('pitch')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(pitch_fly_int),['Corr. coeff. ' num2str(corrcoef_pitch(2,1))])
box off
set(gca,'XColor','none');

subplot(3,3,8)
plot(t_fly_scaled_aligned,turn_dir*(yaw_fly_sp/pi*180)), hold on
plot(t_trans_scaled_aligned,yaw_trans)
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('yaw')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*(yaw_fly_int)),['Corr. coeff. ' num2str(corrcoef_yaw(2,1))])
box off

xlabel('(half)wingbeats')
% legend('fly','robot','location','northwest')


% figure
subplot(3,3,3)
plot(t_fly_scaled_aligned,velx_fly_scaled), hold on
plot(t_trans_scaled_aligned,velx_trans_scaled)
ylabel('vx (R/(half)wingb.)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(velx_fly_scaled),['Corr. coeff. ' num2str(corrcoef_velx(2,1))])
box off
set(gca,'XColor','none');

subplot(3,3,6)
plot(t_fly_scaled_aligned,turn_dir*vely_fly_scaled), hold on
plot(t_trans_scaled_aligned,vely_trans_scaled)
ylabel('vy (R/(half)wingb.)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*vely_fly_scaled),['Corr. coeff. ' num2str(corrcoef_vely(2,1))])
box off
set(gca,'XColor','none');

subplot(3,3,9)
plot(t_fly_scaled_aligned,velz_fly_scaled), hold on
plot(t_trans_scaled_aligned,velz_trans_scaled)
xlabel('(half)wingbeats')
ylabel('vz (R/(half)wingb.)')
% legend('fly','robot')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(velz_fly_scaled),['Corr. coeff. ' num2str(corrcoef_velz(2,1))])
box off

figure('Position',[100 100 800 360])
subplot(3,3,1)
plot(t_fly_scaled_aligned,turn_dir*bet_fly), hold on
plot(t_trans_scaled_aligned,bet_trans)
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('sideslip')
box off
set(gca,'XColor','none');

% legend('fly','robot')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*bet_fly),['Corr. coeff. ' num2str(corrcoef_bet(2,1))])
ylim([-20 80])

subplot(3,3,4)
plot(t_fly_scaled_aligned,turn_dir*course_fly/pi*180), hold on
plot(t_trans_scaled_aligned,course_trans)
ca=gca;
ca.YAxis.TickLabelFormat = '%g\\circ';
ylabel('course')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(turn_dir*course_fly/pi*180),['Corr. coeff. ' num2str(corrcoef_course(2,1))])
ylim([-20 120])
box off
set(gca,'XColor','none');

subplot(3,3,7)
plot(t_fly_scaled_aligned,VEL_fly_scaled), hold on
plot(t_trans_scaled_aligned,VEL_trans_scaled)
ylabel('speed (R/(half)wingb.)')
xlim(t_fly_scaled1([1,end])-t_fly_scaled1(1))
% text(0.1*t_fly_int(end),max(VEL_fly_scaled),['Corr. coeff. ' num2str(corrcoef_VEL(2,1))])
xlabel('(half)wingbeats')
box off

ind_trans=1:length(t_trans_scaled_aligned);
ind_min=ind_trans(t_trans_scaled_aligned>0);
ind_min=ind_min(1);
ind_max=ind_trans(t_trans_scaled_aligned<(t_fly_scaled1(end)-t_fly_scaled1(1)));
ind_max=ind_max(end);

% figure
subplot(3,3,[2,3,5,6,8,9])
plot3(posx_fly_scaled,turn_dir*posy_fly_scaled,posz_fly_scaled),hold on
plot3(posx_trans_scaled(ind_min:ind_max),posy_trans_scaled(ind_min:ind_max),posz_trans_scaled(ind_min:ind_max))
plot3(posx_trans_scaled(ind_min),posy_trans_scaled(ind_min),posz_trans_scaled(ind_min),'ko')
view([0,0,1])
axis equal
% grid
xlabel('x (wing length)')
ylabel('y (wing length)')
zlabel('z (wing length)')
legend('fly','robot')
set(gca,'YDir','reverse')
set(gca,'ZDir','reverse')
box off

%% export correlation and rmse data
export_vars={'posx','posy','posz','omx','omy','omz','velx','vely','velz','course','bet','VEL','roll','pitch','yaw'};

AA=[];
for i=1:length(export_vars)
    eval(['AA=[AA; corrcoef_' char(export_vars(i)) '(2,1), ' char(export_vars(i)) '_rmse_norm];'])
        
end

AA
