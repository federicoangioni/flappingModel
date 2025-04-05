Izz=3.54e-5;
ly = (28.4+70)/1000;
lz = -27/1000;

% Thrust = c1*f + c2 (single wing !!!), values according to force balance measurements in the Science paper
c1 = 0.0107;
c2 = -0.03925;

% bx0 = 0.0722; % Aerodynamic damping, estimated from windtunnel flight by Karl at nominal flapping frequency
bx0 = 0.0722/2; %for Science paper. We would get the same effect by multiplying Izz by a factor of 2 (more realistic, but we would then need to describe the wind tunnel tests)

g = 9.81;
m=29.85e-3; % complete setup, with markers, standing gear and a battery

f0=(m*g/2-c2)/c1; % nominal flapping frequency estimate
bx=bx0/f0;
ampl=44/180*pi;
bx_ampl=bx0/f0/ampl

Cyaw0=2*bx*f0*ly^2/Izz;

UD=[];
VD=[];
WD=[];

PD=[];
QD=[];
RD=[];

U=[];
V=[];
W=[];

P=[];
Q=[];
R=[];

PSI=[];
THETA=[];
PHI=[];

FL=[];
FR=[];
FR_MEAS=[];
DIHED_MEAS=[];

%%
for i=1:5
    switch i
        case 1
            Nexp=56;
            bat_corr=0.4;
        case 2
            Nexp=55;
            bat_corr=0.5;
        case 3
            Nexp=58;
            bat_corr=0.75;
        case 4
            Nexp=54;
            bat_corr=0.25;
        case 5
            Nexp=57;
            bat_corr=0.8;
    end
    
    %% load data
    eval(['data=experiment' num2str(Nexp) ';']);
    
    assign_repetition_variables
    
    u = mean(VELx(:,1.5*120:2.5*120),'omitnan');
    v = mean(VELy(:,1.5*120:2.5*120),'omitnan');
    w = mean(VELz(:,1.5*120:2.5*120),'omitnan');
    ud = mean(VELDx(:,1.5*120:2.5*120),'omitnan');
    vd = mean(VELDy(:,1.5*120:2.5*120),'omitnan');
    wd = mean(VELDz(:,1.5*120:2.5*120),'omitnan');
    p = mean(OMx(:,1.5*120:2.5*120),'omitnan')/180*pi;
    q = mean(OMy(:,1.5*120:2.5*120),'omitnan')/180*pi;
    r = mean(OMz(:,1.5*120:2.5*120),'omitnan')/180*pi;
    pd = mean(ALPHx(:,1.5*120:2.5*120),'omitnan')/180*pi;
    qd = mean(ALPHy(:,1.5*120:2.5*120),'omitnan')/180*pi;
    rd = mean(ALPHz(:,1.5*120:2.5*120),'omitnan')/180*pi;
    gam_cmd = mean(CMDpitch(:,1.5*120:2.5*120),'omitnan')/100*18/180*pi;
    gam_meas = mean(DIHED(:,1.5*120:2.5*120),'omitnan')/180*pi;
    gamd_meas = [0 diff(gam_meas)];
    gam_trim=mean(gam_meas(1:20),'omitnan');
    if isnan(gam_trim)
        gam_trim=0;
    end
    f = mean(FREQ(:,1.5*120:2.5*120),'omitnan');
    phi = mean(ROLL(:,1.5*120:2.5*120),'omitnan')/180*pi;
    theta = mean(PITCH(:,1.5*120:2.5*120),'omitnan')/180*pi;
    psi = mean(YAW(:,1.5*120:2.5*120),'omitnan')/180*pi;
    
    TIMEendOL=TIME(TIME>TIMEman);
    TIMEendOL=TIMEendOL(1);
    OMxEndOL=OMx(:,TIME==TIMEendOL);
    OMyEndOL=OMy(:,TIME==TIMEendOL);
    
    q2p_endOL(i)=mean(OMyEndOL./OMxEndOL);
    
    
    %% actuator dynamics (identified by Karl)
    
    % f = s1*cmd + s2, values according to single wing force balance measurements
    s1 = 0.2014;
    s2 = 3.9517*bat_corr; % 90% correction to fit the data
    
    [A,B,C,D]=tf2ss([12.56],[1 12.56]);
    sys_motor=ss(A,B,C,D);
    fL_cmd=CMDleftavg(1.5*120:2.5*120)*s1+s2-0.5*w+0.5*v; % flapping frequency decrease due to w
    fL=lsim(sys_motor,fL_cmd,TIME(1.5*120:2.5*120)-TIME(1.5*120),f0/C)';
    
    fR_cmd=CMDrightavg(1.5*120:2.5*120)*s1+s2-0.5*w-0.5*v; % flapping frequency decrease due to w
    fR=lsim(sys_motor,fR_cmd,TIME(1.5*120:2.5*120)-TIME(1.5*120),f0/C)';
    
    %%
    
    
    UD=[UD, ud'];
    VD=[VD, vd'];
    WD=[WD, wd'];
    
    PD=[PD, pd'];
    QD=[QD, qd'];
    RD=[RD, rd'];
    
    U=[U, u'];
    V=[V, v'];
    W=[W, w'];
    
    P=[P, p'];
    Q=[Q, q'];
    R=[R, r'];
    
    PSI=[PSI, psi'];
    THETA=[THETA, theta'];
    PHI=[PHI, phi'];
    
    FL=[FL, fL'];
    FR=[FR, fR'];
    FR_MEAS=[FR_MEAS, f'];
    DIHED_MEAS=[DIHED_MEAS, gam_meas'];
    TIME=TIME(1.5*120:2.5*120)';
    
    
end
