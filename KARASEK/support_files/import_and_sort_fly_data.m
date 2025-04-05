% dataset from Science Supplement
load FRUITFLY_LOOMINGRESPONSE_DATABASE.mat

% extra data from Florian extracted from:
% - wingNbodyKin_fruitfly_evsiveManeuvers_4clusers.mat (sorting to bins: n1, n2, n3, n4)
% - kinflightpathDB_pos_qbodyEKF_INCroll_2clusters_Ahor2.75mps2_strokeplane47.5deg_startframe2945.mat (response sample numbers: n_first, n_resp)
load FRUITFLY_extra_data.mat

NEXP=size(body_data.pos,2);
turn_dir=1;

Vars={'VELX','VELY','VELZ',...
      'DVELX','DVELY','DVELZ',...
      'OMX','OMY','OMZ',...
      'ALPHX','ALPHY','ALPHZ',...
      'ROLL','PITCH','YAW',...
      'POSx','POSy','POSz',...
      'POSx_aligned','POSy_aligned','POSz_aligned',...
      'STROKEL','DEVL','PITCHL',...
      'STROKER','DEVR','PITCHR',...
      'COURSE','BET','VELOC','TIME'};
vars={'velx_fly_sp','vely_fly_sp','velz_fly_sp',...
      'dvelx_fly_sp','dvely_fly_sp','dvelz_fly_sp',...
      'omx_fly_sp','omy_fly_sp','omz_fly_sp',...
      'alphx_fly_sp','alphy_fly_sp','alphz_fly_sp',...
      'roll_fly_sp','pitch_fly_sp','yaw_fly_sp',...
      'posx_fly','posy_fly','posz_fly',...
      'posx_fly_aligned','posy_fly_aligned','posz_fly_aligned',...
      'stroke_L','dev_L','pitch_L',...
      'stroke_R','dev_R','pitch_R',...
      'course_fly','bet_fly_sp','VEL_fly','time_fly1'};

for ii=1:length(Vars)
    eval([char(Vars(ii)) '=[];'])
end

PITCH2ROLL=NaN(1,NEXP);

for Nexp_fly=1:NEXP

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

% Muijres et al. 2015 Body saccades of Drosophila consist of stereotyped banked turns"
Ixx_fly=0.64*m_fly*g*R_fly/f_fly^2;
Iyy_fly=1.07*m_fly*g*R_fly/f_fly^2;
Izz_fly=0.57*m_fly*g*R_fly/f_fly^2;

Croll_fly=0.22*m_fly*g*R_fly/f_fly;
Cpitch_fly=0.08*m_fly*g*R_fly/f_fly;
Cyaw_fly=0.41*m_fly*g*R_fly/f_fly;

%% fly data
% wing kinematics
stroke_L=wing_data.stroke_L(:,Nexp_fly);
pitch_L=wing_data.pitch_L(:,Nexp_fly);
dev_L=wing_data.dev_L(:,Nexp_fly);
stroke_R=wing_data.stroke_R(:,Nexp_fly);
pitch_R=wing_data.pitch_R(:,Nexp_fly);
dev_R=wing_data.dev_R(:,Nexp_fly);

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
course_fly=atan2(vely_fly,velx_fly);

Nsamples=length(time_fly);
index=1:Nsamples;

% keep only tracked samples
posx_fly_tracked=posx_fly(~isnan(posx_fly));
posy_fly_tracked=posy_fly(~isnan(posy_fly));
posz_fly_tracked=posz_fly(~isnan(posz_fly));

time0_fly=time_fly(~isnan(posx_fly));
index_tracked=index(~isnan(posx_fly));

dt=time0_fly(2)-time0_fly(1);

% reset position and heading at the beginning
NN=20;
nresp=Nresp(Nexp_fly);
nfirst=Nfirst(Nexp_fly);

heading0(Nexp_fly)=atan2(posy_fly(nfirst+20)-posy_fly(nfirst),posx_fly(nfirst+20)-posx_fly(nfirst));
% heading0(Nexp_fly)=course_fly(nfirst);

posx_fly_aligned=(posx_fly-posx_fly_tracked(1))*cos(heading0(Nexp_fly))+(posy_fly-posy_fly_tracked(1))*sin(heading0(Nexp_fly));
posy_fly_aligned=(-(posx_fly-posx_fly_tracked(1))*sin(heading0(Nexp_fly))+(posy_fly-posy_fly_tracked(1))*cos(heading0(Nexp_fly)));
posz_fly_aligned=-(posz_fly-posz_fly_tracked(1));

% figure
% plot3(posx_fly_aligned,posy_fly_aligned,posz_fly_aligned), hold on
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

% differentiate to get accelerations
alphx_fly_sp=[NaN; diff(omx_fly_sp)/dt];
alphy_fly_sp=[NaN; diff(omy_fly_sp)/dt];
alphz_fly_sp=[NaN; diff(omz_fly_sp)/dt];

dvelx_fly_sp=[NaN; diff(velx_fly_sp)/dt];
dvely_fly_sp=[NaN; diff(vely_fly_sp)/dt];
dvelz_fly_sp=[NaN; diff(velz_fly_sp)/dt];

% correct for the reference frame offset
yaw_fly_sp=yaw_fly_sp-45/180*pi;
course_fly=course_fly-45/180*pi;

% keep yaw at the beginning of the manuever within +/- 180 deg
if ~isnan(yaw_fly_sp(nresp-300))
    while yaw_fly_sp(nresp-300)>pi
        yaw_fly_sp=yaw_fly_sp-2*pi;
    end
    while yaw_fly_sp(nresp-300)<-pi
        yaw_fly_sp=yaw_fly_sp+2*pi;
    end
else
    while yaw_fly_sp(index_tracked(1))>pi
        yaw_fly_sp=yaw_fly_sp-2*pi;
    end
    while yaw_fly_sp(index_tracked(1))<-pi
        yaw_fly_sp=yaw_fly_sp+2*pi;
    end
end

% keep the course at beginning of the manuever within <-180,+180> deg
if ~isnan(course_fly(nresp-300))
    while course_fly(nresp-300)>pi
        course_fly=course_fly-2*pi;
    end
    while course_fly(nresp-300)<-pi
        course_fly=course_fly+2*pi;
    end
else
    while course_fly(index_tracked(1))>pi
        course_fly=course_fly-2*pi;
    end
    while course_fly(index_tracked(1))<-pi
        course_fly=course_fly+2*pi;
    end
end

% making yaw and course continuous
yaw_fly_sp=make_angle_continuous(yaw_fly_sp)-45/180*pi;
course_fly=make_angle_continuous(course_fly)-45/180*pi;

% zero the course at the beginning
course_fly1=course_fly(nresp-300:nresp+300);
course_fly1=course_fly1(~isnan(course_fly1));
course_fly=course_fly-course_fly1(1);
course_fly1=course_fly1-course_fly1(1);

yaw_fly1=yaw_fly_sp(nresp-300:nresp+300);
yaw_fly1=yaw_fly1(~isnan(yaw_fly1));
delta_yaw(Nexp_fly)=yaw_fly1(end)-yaw_fly1(1);

% invert signs to make all turns right using omx (gives the same rate results, as the 2014 Science paper)
yaw_fly_sp=yaw_fly_sp*sign(omx_fly_sp_scaled(nresp));
omz_fly_sp=omz_fly_sp*sign(omx_fly_sp_scaled(nresp));
omz_fly_sp_scaled=omz_fly_sp_scaled*sign(omx_fly_sp_scaled(nresp));
alphz_fly_sp=alphz_fly_sp*sign(omx_fly_sp_scaled(nresp));

course_fly=course_fly*sign(omx_fly_sp_scaled(nresp));
course_fly1=course_fly1*sign(omx_fly_sp_scaled(nresp));

posy_fly=posy_fly*sign(omx_fly_sp_scaled(nresp));
posy_fly_aligned=posy_fly_aligned*sign(omx_fly_sp_scaled(nresp));
vely_fly_sp=vely_fly_sp*sign(omx_fly_sp_scaled(nresp));
dvely_fly_sp=dvely_fly_sp*sign(omx_fly_sp_scaled(nresp));

roll_fly_sp=roll_fly_sp*sign(omx_fly_sp_scaled(nresp));
omx_fly_sp=omx_fly_sp*sign(omx_fly_sp_scaled(nresp));
alphx_fly_sp=alphx_fly_sp*sign(omx_fly_sp_scaled(nresp));

omx_fly_sp_scaled=omx_fly_sp_scaled*sign(omx_fly_sp_scaled(nresp));

VEL_fly=(velx_fly_sp.^2+vely_fly_sp.^2+velz_fly_sp.^2).^0.5;
bet_fly_sp=asind(vely_fly_sp./VEL_fly);

PITCH2ROLL(Nexp_fly)=omy_fly_sp(nresp)/omx_fly_sp(nresp);


% keep the course within <-90,+270> deg
for jj=1:length(course_fly)
    while course_fly(jj)>3*pi/2
        course_fly(jj)=course_fly(jj)-2*pi;
    end
    while course_fly(jj)<-pi/2
        course_fly(jj)=course_fly(jj)+2*pi;
    end
end

time_fly1=time_fly-time_fly(nresp);

% save data, clip to keep only the segments -300:+300 samples from the center of the turn
for ii=1:length(Vars)
    eval([char(Vars(ii)) '=[' char(Vars(ii)) ' ' char(vars(ii)) '(nresp-300:nresp+300)];'])
end



end


% compute q2p ratios for the binned data
time0=0.005;

% experiment numbers belonging to the 4 bins
N45=n1;
N90=n2;
N135=n3;
N180=n4;

% for jj=1:4
%     eval(['PITCH2ROLL' num2str(jj) '=max(mean(OMY(:,n' num2str(jj) '),2,''omitnan''))/max(mean(OMX(:,n' num2str(jj) '),2,''omitnan''));'])
% end

SaveVars=Vars;
for jj=1:4
    for ii=1:length(Vars)
        eval([char(Vars(ii)) num2str(jj) '=[' char(Vars(ii)) '(:,n' num2str(jj) ')];'])
        eval(['SaveVars=[SaveVars ''' char(Vars(ii)) num2str(jj) '''];']) 
    end
    eval(['PITCH2ROLL' num2str(jj) '=max(mean(OMY(:,n' num2str(jj) '),2,''omitnan''))/max(mean(OMX(:,n' num2str(jj) '),2,''omitnan''));'])
end

% SaveVars=[SaveVars 'PITCH2ROLL1' 'PITCH2ROLL2' 'PITCH2ROLL3' 'PITCH2ROLL4'];

% save('fly_maneuvers_sorted_data.mat',SaveVars{:})