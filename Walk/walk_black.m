function Joint=walk_black()

robot=black_robot;
robot=fkinematic(robot,1);
walk_para.L_step=0.0;
walk_para.N_step=18;
walk_para.T_step=0.7;
walk_para.P_double=0.2;
walk_para.T_ready=1.5;
walk_para.Ts=0.005;
walk_para.H_step=0.06;
walk_para.g=9.8;
walk_para.zc=0.62;

robot(3).q=10/180*pi;
robot(5).q=-20.95048383/180*pi;
robot(6).q=10.95048383/180*pi;

robot(10).q=10/180*pi;
robot(12).q=-20.95048383/180*pi;
robot(13).q=10.95048383/180*pi;
robot=fkinematic(robot,1);
walk_para.H_reset=-robot(8).p(3);

Tra_ankle=Tra_ankle_gen_walk(robot,walk_para);
Tra_ZMP=Tra_ZMP_gen_walk(robot,walk_para);
Tra_CoM=Tra_CoM_gen_walk(Tra_ZMP,robot,walk_para);

% plot(Tra_ZMP.x)
% hold on
% plot(Tra_CoM.x)

Joint=zeros(length(Tra_ZMP.x),21);
Joint(:,16)=-0/180*pi;
Joint(:,18)=50/180*pi;
Joint(:,19)=-0/180*pi;
Joint(:,21)=50/180*pi;
for ii=1:length(Tra_ZMP.x)
    TargetR.p=[Tra_ankle.xR(ii);Tra_ankle.yR(ii);Tra_ankle.zR(ii)]-[Tra_CoM.x(ii);Tra_CoM.y(ii);Tra_CoM.z(ii)];
    TargetL.p=[Tra_ankle.xL(ii);Tra_ankle.yL(ii);Tra_ankle.zL(ii)]-[Tra_CoM.x(ii);Tra_CoM.y(ii);Tra_CoM.z(ii)];
    TargetR.R=eye(3);
    TargetL.R=eye(3);
    robot=ikinematic(robot,7,TargetR);
    robot=ikinematic(robot,14,TargetL);
    
    Joint(ii,1)=robot(2).q;
    Joint(ii,2)=robot(4).q;
    Joint(ii,3)=robot(3).q;
    Joint(ii,4)=robot(5).q;
    Joint(ii,5)=robot(6).q;
    Joint(ii,6)=robot(7).q;
    Joint(ii,7)=robot(9).q;
    Joint(ii,8)=robot(11).q;
    Joint(ii,9)=robot(10).q;
    Joint(ii,10)=robot(12).q;
    Joint(ii,11)=robot(13).q;
    Joint(ii,12)=robot(14).q;
    
end

q_mark=[
    0 0 9.951  -20.848  10.897  0 0 0 9.951  -20.848  10.897  0 0 0 0  -0.58  0 0   -0.58 0 0;
    Joint(1,:)/pi*180;
];

Joint_1=bothsides_differ_spline(q_mark,2,0.005)/180*pi;

Joint=[Joint_1;Joint];
end



