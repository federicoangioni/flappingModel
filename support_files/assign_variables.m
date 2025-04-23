time=data.motion_tracking.TIME;

posX=data.motion_tracking.POSx(nman,:);
posY=data.motion_tracking.POSy(nman,:);
posZ=data.motion_tracking.POSz(nman,:);

posX_aligned=data.motion_tracking.POSx_aligned(nman,:);
posY_aligned=data.motion_tracking.POSy_aligned(nman,:);
posZ_aligned=data.motion_tracking.POSz_aligned(nman,:);

omx=data.motion_tracking.OMx_filtered(nman,:);
omy=data.motion_tracking.OMy_filtered(nman,:);
omz=data.motion_tracking.OMz_filtered(nman,:);

alphx=data.motion_tracking.ALPHx_filtered(nman,:);
alphy=data.motion_tracking.ALPHy_filtered(nman,:);
alphz=data.motion_tracking.ALPHz_filtered(nman,:);

head=data.motion_tracking.COURSE(nman,:);
head_aligned=data.motion_tracking.COURSE_aligned(nman,:);
dhead=data.motion_tracking.TURN_RATE(nman,:);

roll=data.motion_tracking.ROLL(nman,:);
pitch=data.motion_tracking.PITCH(nman,:);
yaw=data.motion_tracking.YAW(nman,:);
yaw_aligned=data.motion_tracking.YAW_aligned(nman,:);

bet=data.motion_tracking.SIDESLIP(nman,:);
veloc=data.motion_tracking.SPEED(nman,:);
dihed=data.motion_tracking.DIHEDRAL(nman,:);

cmd_hingeL_interp=data.onboard_interpolated.CMDpitch_filtered_interp(nman,:)/100*18/180*pi;
cmd_hingeR_interp=-data.onboard_interpolated.CMDpitch_filtered_interp(nman,:)/100*18/180*pi;

cmd_motorL_interp=data.onboard_interpolated.CMDleft_motor_filtered_interp(nman,:);
cmd_motorR_interp=data.onboard_interpolated.CMDright_motor_filtered_interp(nman,:);

cmd_roll_interp=data.onboard_interpolated.CMDroll_filtered_interp(nman,:);
cmd_pitch_interp=data.onboard_interpolated.CMDpitch_filtered_interp(nman,:);
cmd_yaw_interp=data.onboard_interpolated.CMDyaw_filtered_interp(nman,:);
cmd_thrust_interp=data.onboard_interpolated.CMDthrottle_filtered_interp(nman,:);

time_onboard=cell2mat(data.onboard.angles_commands_setpoints.TIME_onboard(nman,:));

cmd_pitch=cell2mat(data.onboard.angles_commands_setpoints.CMDpitch(nman,:));
cmd_roll=cell2mat(data.onboard.angles_commands_setpoints.CMDroll(nman,:));
cmd_yaw=cell2mat(data.onboard.angles_commands_setpoints.CMDyaw(nman,:));
cmd_thrust=cell2mat(data.onboard.angles_commands_setpoints.CMDthrottle(nman,:));

cmd_pitch_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDpitch_filtered(nman,:));
cmd_roll_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDroll_filtered(nman,:));
cmd_yaw_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDyaw_filtered(nman,:));
cmd_thrust_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDthrottle_filtered(nman,:));
cmd_hingeL_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDpitch_filtered(nman,:))/100*18/180*pi;
cmd_hingeR_filtered=-cell2mat(data.onboard.angles_commands_setpoints.CMDpitch_filtered(nman,:))/100*18/180*pi;
cmd_motorL_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDleft_motor_filtered(nman,:));
cmd_motorR_filtered=cell2mat(data.onboard.angles_commands_setpoints.CMDright_motor_filtered(nman,:));



rc_pitch=cell2mat(data.onboard.angles_commands_setpoints.SETpitch(nman,:));
rc_roll=cell2mat(data.onboard.angles_commands_setpoints.SETroll(nman,:));
rc_yaw=cell2mat(data.onboard.angles_commands_setpoints.SETyaw(nman,:));

pitch_onboard=cell2mat(data.onboard.angles_commands_setpoints.PITCH_IMU(nman,:));
roll_onboard=cell2mat(data.onboard.angles_commands_setpoints.ROLL_IMU(nman,:));
yaw_onboard=cell2mat(data.onboard.angles_commands_setpoints.YAW_IMU(nman,:));

time_onboard_rates=cell2mat(data.onboard.rates.TIME_onboard_rates(nman,:));
roll_rate_onboard=cell2mat(data.onboard.rates.OMx_IMU_filtered(nman,:));
pitch_rate_onboard=cell2mat(data.onboard.rates.OMy_IMU_filtered(nman,:));
yaw_rate_onboard=cell2mat(data.onboard.rates.OMz_IMU_filtered(nman,:));
roll_acc_onboard=cell2mat(data.onboard.rates.ALPHx_IMU_filtered(nman,:));
pitch_acc_onboard=cell2mat(data.onboard.rates.ALPHy_IMU_filtered(nman,:));
yaw_acc_onboard=cell2mat(data.onboard.rates.ALPHz_IMU_filtered(nman,:));

time_freq=cell2mat(data.onboard.frequency.TIME_onboard_freq(nman,:));
freq_right=cell2mat(data.onboard.frequency.FREQright_wing(nman,:));

vel_CG_D=[data.motion_tracking.VEL_BODYx_filtered(nman,:);...
          data.motion_tracking.VEL_BODYy_filtered(nman,:);...
          data.motion_tracking.VEL_BODYz_filtered(nman,:)];

vel_CG_E=[data.motion_tracking.VEL_GROUNDx_filtered(nman,:);...
          data.motion_tracking.VEL_GROUNDy_filtered(nman,:);...
          data.motion_tracking.VEL_GROUNDz_filtered(nman,:)];

vel_CG_E_aligned=[data.motion_tracking.VEL_GROUNDx_filtered_aligned(nman,:);...
                  data.motion_tracking.VEL_GROUNDy_filtered_aligned(nman,:);...
                  data.motion_tracking.VEL_GROUNDz_filtered_aligned(nman,:)];

acc_CG_E_x=data.motion_tracking.ACC_GROUNDx_filtered(nman,:);
acc_CG_E_y=data.motion_tracking.ACC_GROUNDy_filtered(nman,:);
acc_CG_E_z=data.motion_tracking.ACC_GROUNDz_filtered(nman,:);

acc_CG_E_aligned_x=data.motion_tracking.ACC_GROUNDx_filtered_aligned(nman,:);
acc_CG_E_aligned_y=data.motion_tracking.ACC_GROUNDy_filtered_aligned(nman,:);
acc_CG_E_aligned_z=data.motion_tracking.ACC_GROUNDz_filtered_aligned(nman,:);

acc_CG_D_x=data.motion_tracking.ACC_BODYx_filtered(nman,:);
acc_CG_D_y=data.motion_tracking.ACC_BODYy_filtered(nman,:);
acc_CG_D_z=data.motion_tracking.ACC_BODYz_filtered(nman,:);

TIMEman=data.motion_tracking.TIMEman;
