%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function delfly_animation_simple(posx,posy,posz,roll,pitch,yaw,time,anim_opts)
%
% DelFly Transformer animation
% April 2017, Matej Karasek (matejkarasek@gmail.com)
%
% optional arguments:
% - 'COG' [x;y;z], center of gravity position in meters
% - 'anim_start', first animation frame
% - 'anim_step', step between animation frames
% - 'anim_view', animation view 'top', 'front', 'side' or 'iso' (default)
% - 'anim_traces', leave position traces? 'on', 'off'
% - 'anim_scale', scale of DelFly model (1 - real dimensions)
% - 'anim_detail', zoom on and follow DelFly? options: 'on', 'off'
% - 'anim_lims' [xmin,xmax,ymin,ymax,zmin,zmax] animation limits in meters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function transformer_sequence(posx,posy,posz,roll,pitch,yaw,dihedrL,dihedrR,cmd_motor_left,cmd_motor_right,cmd_yaw,forces,moments,speed,time,mass,anim_opts)

anim_beek='on';
anim_dot='off';
anim_shift='lateral';

% parse animation options
%%%%%%%%%%%%%%%%%%%%%%%%%
if exist('anim_opts','var')
    for i=1:2:length(anim_opts)-1
        switch char(anim_opts(i))
            case 'COG', COG=cell2mat(anim_opts(i+1));
            case 'anim_start',anim_start=cell2mat(anim_opts(i+1));
            case 'anim_step', anim_step=cell2mat(anim_opts(i+1));
            case 'anim_length', anim_length=cell2mat(anim_opts(i+1));
            case 'anim_view', anim_view=char(anim_opts(i+1));
            case 'anim_traces', anim_traces=char(anim_opts(i+1));
            case 'anim_scale', anim_scale=cell2mat(anim_opts(i+1));
            case 'anim_detail', anim_detail=char(anim_opts(i+1));
            case 'anim_delfly', anim_delfly=char(anim_opts(i+1));
            case 'anim_control', anim_control=char(anim_opts(i+1));
            case 'anim_force', anim_force=char(anim_opts(i+1));
            case 'anim_moment', anim_moment=char(anim_opts(i+1));
            case 'anim_speed',anim_speed=char(anim_opts(i+1));
            case 'anim_align', anim_align=cell2mat(anim_opts(i+1));
            case 'anim_zero', anim_zero=cell2mat(anim_opts(i+1));
            case 'anim_lims', anim_lims=cell2mat(anim_opts(i+1));
        end
    end
end
                             
% defaults for optional arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% animation settings
if ~exist('COG','var'), COG=[0;0;-0.06]; end
if ~exist('anim_start','var'), anim_start=5000; end
if ~exist('anim_step','var'), anim_step=100; end 
if ~exist('anim_length','var'), anim_length=0; end 
if ~exist('anim_view','var'), anim_view='iso'; end
if ~exist('anim_traces','var'), anim_traces='on'; end
if ~exist('anim_scale','var'), anim_scale=1; end 
if ~exist('anim_detail','var'), anim_detail='off'; end
if ~exist('anim_delfly','var'), anim_delfly='on'; end 
if ~exist('anim_control','var'), anim_control='on'; end 
if ~exist('anim_force','var'), anim_force='on'; end 
if ~exist('anim_moment','var'), anim_moment='off'; end 
if ~exist('anim_speed','var'), anim_speed='on'; end 
if ~exist('anim_align','var'), anim_align='off'; end
if ~exist('anim_zero','var'), anim_zero=0; end


if anim_length==0
    anim_length=length(time)-anim_start-1; % play till the end
end

% size of the plotted volume (in meters)
if exist('anim_lims','var')
    xmin=anim_lims(1);
    xmax=anim_lims(2);
    ymin=anim_lims(3);
    ymax=anim_lims(4);
    zmin=anim_lims(5);
    zmax=anim_lims(6);
else
    xmin=-5;
    xmax=5;
    ymin=-4;
    ymax=4;
    zmin=-1;
    zmax=4;
end

if strcmp(anim_detail,'on')
    xmin=-anim_scale*0.4;
    xmax=anim_scale*0.4;
    ymin=-anim_scale*0.4;
    ymax=anim_scale*0.4;
    zmin=-anim_scale*0.4;
    zmax=anim_scale*0.4;
end

anim_length=round(anim_length);

% DelFly model
%%%%%%%%%%%%%%

% DelFly dimensions
flap_ang=10/180*pi;
winglength=0.14;
wingrib=0.065;
chord=0.085;
wingoffset=0.02;
beek=0.06;
bodywidth=0.00963;
bodylength=chord;
tail_x=0.075;
tail_y=0.085;
tail_z=0.075;
yawarm=0.025;

if strcmp(anim_control,'on')
    % left/right wing commands
    speedL=round(cmd_motor_left);
    speedL(speedL>100)=100;
    speedL(speedL<0)=0;
    speedL(isnan(speedL))=0;
    speedR=round(cmd_motor_right);
    speedR(speedR>100)=100;
    speedR(speedR<0)=0;
    speedR(isnan(speedR))=0;
    
    % yaw servo
    yawserv=-cmd_yaw/100*45/180*pi;
else
    speedL=0.5*ones(size(cmd_motor_left));
    speedR=0.5*ones(size(cmd_motor_right));
    yawserv=zeros(size(cmd_yaw));
    dihedrL=0*dihedrL;
    dihedrR=0*dihedrR;
end

if strcmp(anim_align,'on')
    heading=atan2d(-speed(2,:),speed(1,:));
    
    posx1=posx-posx(anim_start+anim_zero);
    posy1=posy-posy(anim_start+anim_zero);
    posz=posz-posz(anim_start+anim_zero);
    
    if norm(speed(:,anim_start+anim_zero))>0.5
        posx= posx1*cosd(heading(anim_start+anim_zero))+posy1*sind(heading(anim_start+anim_zero));
        posy=-posx1*sind(heading(anim_start+anim_zero))+posy1*cosd(heading(anim_start+anim_zero));
    
        speed=Rot_z(heading(anim_start+anim_zero)/180*pi)*speed;
        forces=Rot_z(heading(anim_start+anim_zero)/180*pi)*forces;
        moments=Rot_z(heading(anim_start+anim_zero)/180*pi)*moments;
    else
        posx= posx1*cosd(yaw(anim_start+anim_zero))+posy1*sind(yaw(anim_start+anim_zero));
        posy=-posx1*sind(yaw(anim_start+anim_zero))+posy1*cosd(yaw(anim_start+anim_zero));
    
        speed=Rot_z(yaw(anim_start+anim_zero)/180*pi)*speed;
        forces=Rot_z(yaw(anim_start+anim_zero)/180*pi)*forces;
        moments=Rot_z(yaw(anim_start+anim_zero)/180*pi)*moments;
    end
    
    yaw=yaw-yaw(anim_start+anim_zero);
end

%DelFly coordinates
DelFly_x=[  0,           0,              0, 0            , 0  0     , 0             , 0            , 0     , 0           ];
DelFly_y=[  0,           0, -bodywidth/2, bodywidth/2, 0, 0     , -bodywidth/2, bodywidth/2, 0     , 0           ];
DelFly_z=[  beek,        0, 0             , 0            , 0, -chord, -chord        , -chord       , -chord, -bodylength ];

DelFlyBeek_x=[  0,       beek  , 0 ];
DelFlyBeek_y=[  0,       0     , 0 ];
DelFlyBeek_z=[  beek,  3*beek/4, beek/2];

DelFly=anim_scale*([DelFly_x; DelFly_y; DelFly_z]-repmat(COG,1,length(DelFly_x)));
DelFlyBeek=anim_scale*([DelFlyBeek_x; DelFlyBeek_y; DelFlyBeek_z]-repmat(COG,1,length(DelFlyBeek_x)));

% DelFly animation
%%%%%%%%%%%%%%%%%%

% figure
% pause(0.01);
% 
% % maximize figure
% % Note: if this method stops working in a future Matlab version, this blog promises
% % to post a new solution
% % http://undocumentedmatlab.com/blog/minimize-maximize-figure-window
% 
% frame_h = get(handle(gcf),'JavaFrame');
% set(frame_h,'Maximized',1);

    figure
    plot3(0,0,0)
    axis equal

for i=anim_start:anim_step:anim_start+anim_length
    
    DelFlyURWing_x=[  -wingoffset*sin(dihedrR(i)), -(wingoffset*sin(dihedrR(i))+winglength*sin(dihedrR(i)+flap_ang)), -(wingoffset*sin(dihedrR(i))+winglength*sin(dihedrR(i)+flap_ang)), -(wingoffset+wingrib)*sin(dihedrR(i)+flap_ang),  -yawarm*sin(yawserv(i)),     -wingoffset*sin(dihedrR(i))];
    DelFlyURWing_y=[  -wingoffset*cos(dihedrR(i)), -(wingoffset*sin(dihedrR(i))+winglength*cos(dihedrR(i)+flap_ang)), -(wingoffset*sin(dihedrR(i))+winglength*cos(dihedrR(i)+flap_ang)), -(wingoffset+wingrib)*cos(dihedrR(i)+flap_ang),  -yawarm*cos(yawserv(i))+bodywidth/2,     -wingoffset*cos(dihedrR(i))] - bodywidth/2;
    DelFlyURWing_z=[  0,  0,                               -30/88*chord,                       -chord,                           -chord, 0];
    
    DelFlyULWing_x=[  -wingoffset*sin(-dihedrL(i)), -yawarm*sin(-yawserv(i)),       -(wingoffset+wingrib)*sin(-dihedrL(i)+flap_ang),  -(wingoffset*sin(-dihedrL(i))+winglength*sin(-dihedrL(i)+flap_ang)), -(wingoffset*sin(-dihedrL(i))+winglength*sin(-dihedrL(i)+flap_ang)),   -wingoffset*sin(-dihedrL(i))];
    DelFlyULWing_y=[  wingoffset*cos(-dihedrL(i)),  yawarm*cos(-yawserv(i))- bodywidth/2,        (wingoffset+wingrib)*cos(-dihedrL(i)+flap_ang),   (wingoffset*sin(-dihedrL(i))+winglength*cos(-dihedrL(i)+flap_ang)),  (wingoffset*sin(-dihedrL(i))+winglength*cos(-dihedrL(i)+flap_ang)),   wingoffset*cos(-dihedrL(i))] + bodywidth/2;
    DelFlyULWing_z=[  0, -chord,  -chord,                            -30/88*chord,                        0,                                0];
    
    DelFlyDRWing_x=[  -wingoffset*sin(dihedrR(i)), -(wingoffset*sin(dihedrR(i))+winglength*sin(dihedrR(i)-flap_ang)), -(wingoffset*sin(dihedrR(i))+winglength*sin(dihedrR(i)-flap_ang)), -(wingoffset+wingrib)*sin(dihedrR(i)-flap_ang),  -yawarm*sin(yawserv(i)),    -wingoffset*sin(dihedrR(i))];
    DelFlyDRWing_y=[  -wingoffset*cos(dihedrR(i)), -(wingoffset*sin(dihedrR(i))+winglength*cos(dihedrR(i)-flap_ang)), -(wingoffset*sin(dihedrR(i))+winglength*cos(dihedrR(i)-flap_ang)), -(wingoffset+wingrib)*cos(dihedrR(i)-flap_ang),  -yawarm*cos(yawserv(i))+bodywidth/2,     -wingoffset*cos(dihedrR(i))] - bodywidth/2;
    DelFlyDRWing_z=[  0,  0,                               -30/88*chord,                       -chord,                           -chord, 0];
    
    DelFlyDLWing_x=[  -wingoffset*sin(-dihedrL(i)), -yawarm*sin(-yawserv(i)),       -(wingoffset+wingrib)*sin(-dihedrL(i)-flap_ang),  -(wingoffset*sin(-dihedrL(i))+winglength*sin(-dihedrL(i)-flap_ang)),  -(wingoffset*sin(-dihedrL(i))+winglength*sin(-dihedrL(i)-flap_ang)),   -wingoffset*sin(-dihedrL(i))];
    DelFlyDLWing_y=[  wingoffset*cos(-dihedrL(i)),  yawarm*cos(-yawserv(i))- bodywidth/2,        (wingoffset+wingrib)*cos(-dihedrL(i)-flap_ang),   (wingoffset*sin(-dihedrL(i))+winglength*cos(-dihedrL(i)-flap_ang)),  (wingoffset*sin(-dihedrL(i))+winglength*cos(-dihedrL(i)-flap_ang)),    wingoffset*cos(-dihedrL(i))] + bodywidth/2;
    DelFlyDLWing_z=[  0, -chord,  -chord,                            -30/88*chord,                        0,                                 0];
    
    DelFlyRArm_x=[  0, -wingoffset*sin(dihedrR(i))];
    DelFlyRArm_y=[  0, -wingoffset*cos(dihedrR(i))] - bodywidth/2;
    DelFlyRArm_z=[  0,  0];
    
    DelFlyLArm_x=[  0, -wingoffset*sin(-dihedrL(i))];
    DelFlyLArm_y=[  0, wingoffset*cos(-dihedrL(i))] + bodywidth/2;
    DelFlyLArm_z=[  0,  0];
    
    DelFlyYawArm_x=[  -yawarm*sin(-yawserv(i)), -yawarm*sin(yawserv(i))];
    DelFlyYawArm_y=[  yawarm*cos(-yawserv(i)), -yawarm*cos(yawserv(i))];
    DelFlyYawArm_z=[  -chord,  -chord];
    
    DelFlyDRWingT_x=[  -(wingoffset*sin(dihedrR(i))+winglength*sin(dihedrR(i)-flap_ang))];
    DelFlyDRWingT_y=[  -(wingoffset*sin(dihedrR(i))+winglength*cos(dihedrR(i)-flap_ang))] - bodywidth/2;
    DelFlyDRWingT_z=[  0];
        
    DelFlyULWing=anim_scale*([DelFlyULWing_x; DelFlyULWing_y; DelFlyULWing_z]-repmat(COG,1,length(DelFlyULWing_x)));
    DelFlyURWing=anim_scale*([DelFlyURWing_x; DelFlyURWing_y; DelFlyURWing_z]-repmat(COG,1,length(DelFlyURWing_x)));
    DelFlyDLWing=anim_scale*([DelFlyDLWing_x; DelFlyDLWing_y; DelFlyDLWing_z]-repmat(COG,1,length(DelFlyDLWing_x)));
    DelFlyDRWing=anim_scale*([DelFlyDRWing_x; DelFlyDRWing_y; DelFlyDRWing_z]-repmat(COG,1,length(DelFlyDRWing_x)));
    DelFlyLArm=anim_scale*([DelFlyLArm_x; DelFlyLArm_y; DelFlyLArm_z]-repmat(COG,1,length(DelFlyLArm_x)));
    DelFlyRArm=anim_scale*([DelFlyRArm_x; DelFlyRArm_y; DelFlyRArm_z]-repmat(COG,1,length(DelFlyRArm_x)));
    DelFlyYawArm=anim_scale*([DelFlyYawArm_x; DelFlyYawArm_y; DelFlyYawArm_z]-repmat(COG,1,length(DelFlyYawArm_x)));
    DelFlyDRWingT=anim_scale*([DelFlyDRWingT_x; DelFlyDRWingT_y; DelFlyDRWingT_z]-COG);
    
    onesWing=ones(size(DelFlyULWing(1,:)));
    onesBeek=ones(size(DelFlyBeek(1,:)));
    
    Rot=Rot_z(yaw(i)/180*pi)*Rot_y(pitch(i)/180*pi)*Rot_x(roll(i)/180*pi);
    
    DelFly_earth=Rot*DelFly;
    DelFlyULWing_earth=Rot*DelFlyULWing;
    DelFlyURWing_earth=Rot*DelFlyURWing;
    DelFlyDLWing_earth=Rot*DelFlyDLWing;
    DelFlyDRWing_earth=Rot*DelFlyDRWing;
    DelFlyLArm_earth=Rot*DelFlyLArm;
    DelFlyRArm_earth=Rot*DelFlyRArm;
    DelFlyYawArm_earth=Rot*DelFlyYawArm;
    DelFlyBeek_earth=Rot*DelFlyBeek;
    DelFlyDRWingT_earth=Rot*DelFlyDRWingT;

    if strcmp(anim_traces,'on')
%         plot3(posx(anim_start:anim_start+anim_length),posy(anim_start:anim_start+anim_length),posz(anim_start:anim_start+anim_length),'m:','LineWidth',1,'MarkerSize',0.5), hold on
        plot3(posx(anim_start:i),posy(anim_start:i),posz(anim_start:i),'-','LineWidth',1,'MarkerSize',0.5,'Color',[0.3 0.3 0.3]), hold on
        if strcmp(anim_view,'iso')
            plot3(posx(anim_start:anim_start+anim_length),posy(anim_start:anim_start+anim_length),zmin*ones(size(posz(anim_start:anim_start+anim_length))),'k:','LineWidth',1,'MarkerSize',0.5)
            plot3(posx(anim_start:anim_start+anim_length),ymax*ones(size(posy(anim_start:anim_start+anim_length))),posz(anim_start:anim_start+anim_length),'k:','LineWidth',1,'MarkerSize',0.5)
            plot3(xmax*ones(size(posx(anim_start:anim_start+anim_length))),posy(anim_start:anim_start+anim_length),posz(anim_start:anim_start+anim_length),'k:','LineWidth',1,'MarkerSize',0.5)
        end
    end
    
%     colors=colormap(hsv(201));
%     colors=colors(101:-1:1,:);
    colors=viridis(101);
    colormap(colors);
%     colorL=[0 0 1]*speedL(i);
%     colorR=[0 0 1]*speedR(i);

    colorL=colors(speedL(i)+1,:);
    colorR=colors(speedR(i)+1,:);
    
    if strcmp(anim_delfly,'on')
        plot3(posx(i)+DelFly_earth(1,:),posy(i)+DelFly_earth(2,:),posz(i)+DelFly_earth(3,:),'-k'), hold on
        patch(posx(i)+DelFlyULWing_earth(1,:),posy(i)+DelFlyULWing_earth(2,:),posz(i)+DelFlyULWing_earth(3,:),onesWing,'FaceColor',colorL)
        patch(posx(i)+DelFlyURWing_earth(1,:),posy(i)+DelFlyURWing_earth(2,:),posz(i)+DelFlyURWing_earth(3,:),onesWing,'FaceColor',colorR)
        patch(posx(i)+DelFlyDLWing_earth(1,:),posy(i)+DelFlyDLWing_earth(2,:),posz(i)+DelFlyDLWing_earth(3,:),onesWing,'FaceColor',colorL)
        patch(posx(i)+DelFlyDRWing_earth(1,:),posy(i)+DelFlyDRWing_earth(2,:),posz(i)+DelFlyDRWing_earth(3,:),onesWing,'FaceColor',colorR)
        if strcmp(anim_beek,'on')
            patch(posx(i)+DelFlyBeek_earth(1,:),posy(i)+DelFlyBeek_earth(2,:),posz(i)+DelFlyBeek_earth(3,:),onesBeek,'FaceColor','b')
        end
        plot3(posx(i)+DelFlyLArm_earth(1,:),posy(i)+DelFlyLArm_earth(2,:),posz(i)+DelFlyLArm_earth(3,:),'.-k'), hold on
        plot3(posx(i)+DelFlyRArm_earth(1,:),posy(i)+DelFlyRArm_earth(2,:),posz(i)+DelFlyRArm_earth(3,:),'.-k'), hold on
        plot3(posx(i)+DelFlyYawArm_earth(1,:),posy(i)+DelFlyYawArm_earth(2,:),posz(i)+DelFlyYawArm_earth(3,:),'.-k'), hold on
        if strcmp(anim_dot,'on')
            plot3(posx(i)+DelFlyDRWingT_earth(1),posy(i)+DelFlyDRWingT_earth(2),posz(i)+DelFlyDRWingT_earth(3),'.k','MarkerSize',15)
        end
    end
        
    
    if strcmp(anim_speed,'on')
%         scaleS=0.05*anim_scale; % standard
        scaleS=0.1*anim_scale; % for yaw control Science figure 4, 1 m/s ~ 0.1 m
        scaleS=0.05*anim_scale; % for Science figure 2, 1 m/s ~ 0.05 m
        plot3([posx(i), posx(i)+speed(1,i)*scaleS], [posy(i), posy(i)-speed(2,i)*scaleS], [posz(i), posz(i)-speed(3,i)*scaleS],'LineWidth',2,'Color',[0 1 1]), hold on
%         plot3([posx(i)+speed(1,i)*scaleS], [posy(i)-speed(2,i)*scaleS], [posz(i)-speed(3,i)*scaleS],'.g','MarkerSize',10,'LineWidth',1), hold on
    end 
    
    if strcmp(anim_force,'on')
%         scaleF=0.4*anim_scale; % standard
        scaleF=1/(mass*9.81)*0.2*anim_scale; % for yaw control Science figure 4, 1g ~ 0.2 m
        scaleF=1/(mass*9.81)*0.1*anim_scale; % for Science figure 2, 1g ~ 0.1 m
        plot3([posx(i), posx(i)+forces(1,i)*scaleF], [posy(i), posy(i)-forces(2,i)*scaleF], [posz(i), posz(i)-forces(3,i)*scaleF],'LineWidth',2,'Color',[1 0.2 1]), hold on
%         plot3([posx(i)+forces(1,i)*scaleF], [posy(i)-forces(2,i)*scaleF], [posz(i)-forces(3,i)*scaleF],'.r','MarkerSize',10,'LineWidth',1), hold on
    end
    
    if strcmp(anim_moment,'on')
%         % moments
%         arc_start=pi+pitch(i)/180*pi;%pi-roll(i)/180*pi;
%         arc_end=pi+3*pi/2*moments(2,i)/10e-3+pitch(i)/180*pi;%+pi-roll(i)/180*pi;
%         arc_rad=anim_scale*0.07;
%         
%         if arc_end>arc_start
%             arc_ang=arc_start:1/100:arc_end;
%         else
%             arc_ang=arc_end:1/100:arc_start;
%         end
%         arcx=arc_rad*sin(arc_ang);
%         arcy=-arc_rad*cos(arc_ang);
%         
%         plot3(posx(i)+arcx,posy(i)*ones(size(arcx))+1,posz(i)+arcy,'-m','LineWidth',2)
        
        % moments
        arc_start=pi+pitch(i)/180*pi;%pi-roll(i)/180*pi;
        arc_end=pi+3*pi/2*moments(2,i)/10e-3+pitch(i)/180*pi;%+pi-roll(i)/180*pi;
        arc_rad=anim_scale*0.07;
        
%         if arc_end>arc_start
%             arc_ang=arc_start:1/100:arc_end;
%         else
%             arc_ang=arc_end:1/100:arc_start;
%         end
        arcx=[arc_rad*sin(arc_start) 0 arc_rad*sin(arc_end)];
        arcy=[-arc_rad*cos(arc_start) 0 -arc_rad*cos(arc_end)];
        
        plot3(posx(i)+arcx,posy(i)*ones(size(arcx))-1,posz(i)+arcy,'-k','LineWidth',1)
        
    end
    
%     title(['t = ' num2str(time(i),'%.3f') ' s, pos = [' num2str(posx(i),'%.3f') ' ,' num2str(posy(i),'%.3f') ' ,' num2str(posz(i),'%.3f') '] m, RPY = [' num2str(roll(i),'%.1f') ' ,' num2str(pitch(i),'%.1f') ' ,' num2str(yaw(i),'%.1f') '] deg.  (Ctrl+C to stop)'])
    
    grid
    axis equal
    if strcmp(anim_detail,'on')
        axis([posx(i)+xmin posx(i)+xmax posy(i)+ymin posy(i)+ymax posz(i)+zmin posz(i)+zmax])
    else
        axis([xmin xmax ymin ymax zmin zmax])
    end
    
    switch anim_view
        case 'top'
            view([0,0,1])
        case 'front'
            view([1,0,0])
        case 'back'
            view([-1,0,0])
        case 'side'
            view([0,1,0])
        case 'iso'
            % default view            
    end
    
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    
    hold on
    
    switch anim_shift
        case 'lateral' 
            posy=posy+2.3*winglength;
        
        case 'longitudinal' 
            posx=posx+2.3*winglength;
            
        otherwise
            % do nothing
    end
            
            
        

end

if strcmp(anim_delfly,'on')
        h=colorbar;
        set(h,...
            'Ticks',[0:0.4:2],...
            'Limits',[0.4 2],...
            'TickLabels',{'0 %','20 %','40 %','60 %','80 %','100 %'},...
            'Location','default');
        
    end
    

% close

