close all
clear all
clc

addpath('force_balance_data')
load force_balance_coupling_data.mat
     
%% science plots coupling lines
colors=viridis(5);

figure('pos',[10 110 350 650])
subplot(3,1,1)
hold on
for i=1:5
    plot(roll(:,i),MxAvg(:,i),'-x','Color',colors(i,:))
end
xlabel('roll cmd (%)')
ylabel('roll torque (Nmm)')
legend('cmd_p_i_t_c_h = 80 %','cmd_p_i_t_c_h = 40 %','cmd_p_i_t_c_h = 0 %','cmd_p_i_t_c_h = -40 %','cmd_p_i_t_c_h = -80 %')
set(gca,'Xlim',[-25 20])

subplot(3,1,2)
hold on
for i=1:5
    plot(roll(:,i),FzAvg(:,i),'-x','Color',colors(i,:))
end
xlabel('roll cmd (%)')
ylabel('thrust (N)')
set(gca,'Xlim',[-25 20])

subplot(3,1,3)
hold on
plot(roll(:,3),MxAvg_COM_UP(:,3),'-.bo')
plot(roll(:,3),MxAvg(:,3),'-kx')
plot(roll(:,3),MxAvg_COM_DOWN(:,3),'--rs')
xlabel('roll cmd (%)')
ylabel('roll torque (Nmm)')
legend('CoM 50 mm higher','CoM nominal','CoM 50 mm lower')
set(gca,'Xlim',[-25 20])


figure('pos',[10 110 350 650])
subplot(3,1,1)
hold on
for i=1:5
    plot(pitch(i,:),MyAvg(i,:),'-x','Color',colors(i,:))
end
xlabel('pitch cmd (%)')
ylabel('pitch torque (Nmm)')
legend('cmd_r_o_l_l = -21.5 %','cmd_r_o_l_l = -11.5 %','cmd_r_o_l_l = -1.5 %','cmd_r_o_l_l = 8.5 %','cmd_r_o_l_l = 18.5 %')

subplot(3,1,2)
hold on
for i=1:5
    plot(pitch(i,:),FzAvg(i,:),'-x','Color',colors(i,:))
end
xlabel('pitch cmd (%)')
ylabel('thrust (N)')

subplot(3,1,3)
hold on
plot(pitch(3,:),MyAvg_COM_UP(3,:),'-.bo')
plot(pitch(3,:),MyAvg(3,:),'-kx')
plot(pitch(3,:),MyAvg_COM_DOWN(3,:),'-rs')
xlabel('pitch cmd (%)')
ylabel('pitch torque (Nmm)')


