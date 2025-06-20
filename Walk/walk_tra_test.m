close all;clear;clc;
robot=black_robot;
robot=fkinematic(robot,1);
walk_para.L_step=0.0;
walk_para.N_step=16;
walk_para.T_step=3;
walk_para.P_double=0.2;
walk_para.T_ready=2;
walk_para.Ts=0.005;
walk_para.H_step=0.1;
walk_para.g=9.8;
walk_para.zc=0.64;

robot(3).q=9.85/180*pi;
robot(5).q=-20.08/180*pi;
robot(6).q=10.23/180*pi;
robot=fkinematic(robot,1);
walk_para.H_reset=-robot(8).p(3);

Tra_ankle=Tra_ankle_gen_walk(robot,walk_para);
Tra_ZMP=Tra_ZMP_gen_walk(robot,walk_para);
Tra_CoM=Tra_CoM_gen_walk(Tra_ZMP,robot,walk_para);



% plot3(Tra_ankle.xL,Tra_ankle.yL,Tra_ankle.zL)
% hold on
% plot3(Tra_ankle.xR,Tra_ankle.yR,Tra_ankle.zR)
% hold on
% plot(Tra_ZMP.x,Tra_ZMP.y)
% hold on
% plot(Tra_CoM.x,Tra_CoM.y)
% figure
plot(Tra_ZMP.x)
hold on
plot(Tra_CoM.x)
% figure
% plot(Tra_ZMP.y)
% hold on
% plot(Tra_CoM.y)
% figure
% plot(Tra_ankle.xL)
% hold on
% plot(Tra_ankle.xR)
% hold on
% plot(Tra_ZMP.x)
% figure
% plot(Tra_ankle.yL)
% hold on
% plot(Tra_ankle.yR)
% hold on
% plot(Tra_ZMP.y)
% figure
% plot(Tra_ankle.zL)
% hold on
% plot(Tra_ankle.zR)
% figure
% plot(Tra_CoM.z)