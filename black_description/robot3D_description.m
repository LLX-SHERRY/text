function robot=robot3D_description()
load('under_joint_position.mat');
load('com.mat')
load('Inertia.mat');
load('mass.mat')

com7=(mass(7)*com(:,7)+mass(8)*com(:,8))/(mass(7)+mass(8));
com14=(mass(14)*com(:,14)+mass(15)*com(:,15))/(mass(14)+mass(15));

Inertia(:,:,7)=Inertia(:,:,7)+mass(7)*hat3(com(:,7)-com7)*(hat3(com(:,7)-com7))'...
	+Inertia(:,:,8)+mass(8)*hat3(com(:,8)-com7)*(hat3(com(:,8)-com7))';
Inertia(:,:,14)=Inertia(:,:,14)+mass(14)*hat3(com(:,14)-com14)*(hat3(com(:,14)-com14))'...
	+Inertia(:,:,15)+mass(15)*hat3(com(:,15)-com14)*(hat3(com(:,15)-com14))';

Inertia(:,:,8)=[];
Inertia(:,:,15)=[];

com(:,7)=com7;
com(:,14)=com14;
com(:,[8,15])=[];


mass(7)=mass(7)+mass(8);
mass(14)=mass(14)+mass(15);
mass([8,15])=[];

p=under_joint_position;
p=p.*(abs(p)>8e-4);
p=[-1 0 0;0 -1 0; 0 0 1]*p;
name      =  { 'hip';       'R1';        'R2';        'R3';        'R4';        'R5';        'R6';				'L1';        'L2';         'L3';        'L4';         'L5';         'L6';         'WaistZ';     'WaistX';     'WaistY';     'Ra1';        'Ra2';        'Ra3';        'Ra4';        'La1';        'La2';        'La3';        'La4'       };
id_me     =  [  1            2            3            4            5            6            7					 8            9            10           11            12            13            14            15            16            17            18            19            20            21            22            23            24          ];
id_child  =  [  2            4            5            3            6            7            0					10           11             9           12            13             0            15            16            19            20            17            18             0            24            21            22             0          ];
id_mother =  [  0            1            4            2            3            5            6				     1           10             8            9            11            12             1            14            15            18            19            16            17            22            23            16            21          ];
id_sister =  [  0            8            0            0            0            0            0					14            0             0            0             0             0             0             0             0             0             0            23             0             0             0             0             0          ];
joint_a   =  [  0            0            0            1            1            1            0					 0            0             1            1             1             0             0             1             0             0             0             1             1             0             0             1             1
				0            0            1            0            0            0            1					 0            1             0            0             0             1             0             0             0             0             1             0             0             0             1             0             0
				0            1            0            0            0            0            0					 1            0             0            0             0             0             1             0             0             1             0             0             0             1             0             0             0          ];
joint_b   =  [  0            p(1,1)       p(1,2)       p(1,3)       p(1,4)       p(1,5)       p(1,6)			 p(1,7)       p(1,8)        p(1,9)       p(1,10)       p(1,11)       p(1,12)       p(1,13)       p(1,14)       p(1,15)       0             p(1,17)       p(1,16)       p(1,18)       0             p(1,20)       p(1,19)       p(1,21)
				0            p(2,1)       p(2,2)       p(2,3)       p(2,4)       p(2,5)       p(2,6)			 p(2,7)       p(2,8)        p(2,9)       p(2,10)       p(2,11)       p(2,12)       p(2,13)       p(2,14)       p(2,15)       0             p(2,17)       p(2,16)       p(2,18)       0             p(2,20)       p(2,19)       p(2,21)
				0            p(3,1)       p(3,2)       p(3,3)       p(3,4)       p(3,5)       p(3,6)			 p(3,7)       p(3,8)        p(3,9)       p(3,10)       p(3,11)       p(3,12)       p(3,13)       p(3,14)       p(3,15)       0             p(3,17)       p(3,16)       p(3,18)       0             p(3,20)       p(3,19)       p(3,21)    ];
lb        =  [  0          -68          -90          -60         -135          -74          -40                -55          -40			  -60         -135           -74           -40           -83           -50             0           -20          -107          -120             0           -20            -5          -134             0          ]/180*pi;
ub        =  [  0           52           40           90            0          120           40                 73           90            90            0           120            40            80            -0             0            20             5           180           128            20            97           180           128          ]/180*pi;
			
% lb        =  [  0           0           0            0            0          0           0						0           0			  0         0					0            0           -83            -0             0           -0          -0          -0				0           -20            -5          -134             0          ]/180*pi;
% ub        =  [  0           0           0            0            0          0           0						0           0            0            0					0            0            80            -0             0            0             0           0				 0            20            97           180           128          ]/180*pi;
			
for ii=1:length(id_me)
	%编号
	robot(ii).name   = name(ii);
	robot(ii).id     = id_me(ii);
	robot(ii).child  = id_child(ii);
	robot(ii).mother = id_mother(ii);
	robot(ii).sister = id_sister(ii);
	
	%几何参数
	robot(ii).a      = joint_a(:,ii);
	robot(ii).b      = joint_b(:,ii);
	robot(ii).lb     = lb(ii);
	robot(ii).ub     = ub(ii);
	
	%动态参数
	robot(ii).q = 0;
	robot(ii).p = [0 0 0]';
	robot(ii).R = eye(3);
	
	%碰撞参数
	robot(ii).collision=[];
end
robot=fkinematic(robot,1);
for ii=1:length(id_me)
	%惯量参数
	robot(ii).m  = mass(ii);
	robot(ii).I  = Inertia(:,:,ii);
	robot(ii).c  = com(:,ii)-robot(ii).p-[0;0.05;0.80638];
end

%碰撞点
robot(4).collision(1).name  = 'right_low_butt';
robot(4).collision(1).b     = [0;-0.06;0.035];
robot(4).collision(1).p     = [0;0;0];

robot(10).collision(1).name = 'right_low_butt';
robot(10).collision(1).b    = [0;-0.06;0.035];
robot(10).collision(1).p    = [0;0;0];
%%左腿6
robot(13).collision(1).name = 'left_heel_L';
robot(13).collision(1).b    = [-0.089;-0.105;-0.064];
robot(13).collision(1).p    = [0;0;0];

robot(13).collision(2).name = 'left_heel_R';
robot(13).collision(2).b    = [0.058;-0.105;-0.064];
robot(13).collision(2).p    = [0;0;0];

robot(13).collision(3).name = 'left_toe_L';
robot(13).collision(3).b    = [-0.089;0.145;-0.064];
robot(13).collision(3).p    = [0;0;0];

robot(13).collision(4).name = 'left_toe_R';
robot(13).collision(4).b    = [0.058;0.145;-0.064];
robot(13).collision(4).p    = [0;0;0];
%%右腿6
robot(7).collision(1).name = 'right_heel_L';
robot(7).collision(1).b    = [-0.058;-0.105;-0.064];
robot(7).collision(1).p    = [0;0;0];

robot(7).collision(2).name = 'right_heel_R';
robot(7).collision(2).b    = [0.089;-0.105;-0.064];
robot(7).collision(2).p    = [0;0;0];

robot(7).collision(3).name = 'right_toe_L';
robot(7).collision(3).b    = [-0.058;0.145;-0.064];
robot(7).collision(3).p    = [0;0;0];

robot(7).collision(4).name = 'right_toe_R';
robot(7).collision(4).b    = [0.089;0.145;-0.064];
robot(7).collision(4).p    = [0;0;0];
%%左腿4 膝盖
robot(11).collision(1).name = 'left_knee_L';
robot(11).collision(1).b    = [-0.053;0.059;0];
robot(11).collision(1).p    = [0;0;0];

robot(11).collision(2).name = 'left_knee_R';
robot(11).collision(2).b    = [0.053;0.059;0];
robot(11).collision(2).p    = [0;0;0];
%%右腿4 膝盖
robot(5).collision(1).name = 'right_knee_L';
robot(5).collision(1).b    = [-0.053;0.059;0];
robot(5).collision(1).p    = [0;0;0];

robot(5).collision(2).name = 'right_knee_R';
robot(5).collision(2).b    = [0.053;0.059;0];
robot(5).collision(2).p    = [0;0;0];
%%左手3 小臂+手
robot(24).collision(1).name = 'left_elbow_L1';
robot(24).collision(1).b    = [-0.045;0.045;0];
robot(24).collision(1).p    = [0;0;0];

robot(24).collision(2).name = 'left_elbow_L2';
robot(24).collision(2).b    = [-0.045;-0.045;0];
robot(24).collision(2).p    = [0;0;0];

robot(24).collision(3).name = 'left_elbow_R';
robot(24).collision(3).b    = [0.045;-0.045;0];
robot(24).collision(3).p    = [0;0;0];

robot(24).collision(4).name = 'left_hand_L1';
robot(24).collision(4).b    = [-0.045;0.045;-0.45];
robot(24).collision(4).p    = [0;0;0];

robot(24).collision(5).name = 'left_hand_L2';
robot(24).collision(5).b    = [-0.045;-0.045;-0.45];
robot(24).collision(5).p    = [0;0;0];

robot(24).collision(6).name = 'left_hand_R1';
robot(24).collision(6).b    = [0.045;0.045;-0.45];
robot(24).collision(6).p    = [0;0;0];

robot(24).collision(7).name = 'left_hand_R2';
robot(24).collision(7).b    = [0.045;-0.045;-0.45];
robot(24).collision(7).p    = [0;0;0];
%%右手3 小臂+手
robot(20).collision(1).name = 'right_elbow_R1';
robot(20).collision(1).b    = [0.045;0.045;0];
robot(20).collision(1).p    = [0;0;0];

robot(20).collision(2).name = 'right_elbow_R2';
robot(20).collision(2).b    = [0.045;-0.045;0];
robot(20).collision(2).p    = [0;0;0];

robot(20).collision(3).name = 'right_elbow_L';
robot(20).collision(3).b    = [-0.045;-0.045;0];
robot(20).collision(3).p    = [0;0;0];

robot(20).collision(4).name = 'right_hand_L1';
robot(20).collision(4).b    = [-0.045;0.045;-0.45];
robot(20).collision(4).p    = [0;0;0];

robot(20).collision(5).name = 'right_hand_L2';
robot(20).collision(5).b    = [-0.045;-0.045;-0.45];
robot(20).collision(5).p    = [0;0;0];

robot(20).collision(6).name = 'right_hand_R1';
robot(20).collision(6).b    = [0.045;0.045;-0.45];
robot(20).collision(6).p    = [0;0;0];

robot(20).collision(7).name = 'right_hand_R2';
robot(20).collision(7).b    = [0.045;-0.045;-0.45];
robot(20).collision(7).p    = [0;0;0];
%%左手1 肩
robot(23).collision(1).name = 'left_shoulder_L1';
robot(23).collision(1).b    = [-0.055;0.055;0];
robot(23).collision(1).p    = [0;0;0];

robot(23).collision(2).name = 'left_shoulder_L2';
robot(23).collision(2).b    = [-0.055;-0.055;0];
robot(23).collision(2).p    = [0;0;0];

robot(23).collision(3).name = 'left_shoulder_L3';
robot(23).collision(3).b    = [-0.055;0;0.055];
robot(23).collision(3).p    = [0;0;0];
%%右手1 肩
robot(19).collision(1).name = 'right_shoulder_R1';
robot(19).collision(1).b    = [0.055;0.055;0];
robot(19).collision(1).p    = [0;0;0];

robot(19).collision(2).name = 'right_shoulder_R2';
robot(19).collision(2).b    = [0.055;-0.055;0];
robot(19).collision(2).p    = [0;0;0];

robot(19).collision(3).name = 'right_shoulder_R3';
robot(19).collision(3).b    = [0.055;0;0.055];
robot(19).collision(3).p    = [0;0;0];
%%腰14 躯干
robot(14).collision(1).name = 'body_L1';
robot(14).collision(1).b    = [-0.115;0.085;0.005];
robot(14).collision(1).p    = [0;0;0];

robot(14).collision(2).name = 'body_L2';
robot(14).collision(2).b    = [-0.115;-0.165;0.255];
robot(14).collision(2).p    = [0;0;0];

robot(14).collision(3).name = 'body_L3';
robot(14).collision(3).b    = [-0.115;0.085;0.255];
robot(14).collision(3).p    = [0;0;0];

robot(14).collision(4).name = 'body_L4';
robot(14).collision(4).b    = [-0.115;-0.165;0.005];
robot(14).collision(4).p    = [0;0;0];

robot(14).collision(5).name = 'body_R1';
robot(14).collision(5).b    = [0.115;0.085;0.005];
robot(14).collision(5).p    = [0;0;0];

robot(14).collision(6).name = 'body_R2';
robot(14).collision(6).b    = [0.115;-0.165;0.255];
robot(14).collision(6).p    = [0;0;0];

robot(14).collision(7).name = 'body_R3';
robot(14).collision(7).b    = [0.115;0.085;0.255];
robot(14).collision(7).p    = [0;0;0];

robot(14).collision(8).name = 'body_R4';
robot(14).collision(8).b    = [0.115;-0.165;0.005];
robot(14).collision(8).p    = [0;0;0];

robot(14).collision(9).name = 'head_1';
robot(14).collision(9).b    = [0;0.05;0.41];
robot(14).collision(9).p    = [0;0;0];

robot(14).collision(10).name = 'head_2';
robot(14).collision(10).b    = [0;-0.11;0.41];
robot(14).collision(10).p    = [0;0;0];

robot(14).collision(11).name = 'head_3';
robot(14).collision(11).b    = [0;-0.03;0.49];
robot(14).collision(11).p    = [0;0;0];

robot(14).collision(12).name = 'head_4';
robot(14).collision(12).b    = [-0.08;-0.03;0.41];
robot(14).collision(12).p    = [0;0;0];

robot(14).collision(13).name = 'head_5';
robot(14).collision(13).b    = [0.08;-0.03;0.41];
robot(14).collision(13).p    = [0;0;0];

robot(14).collision(14).name = 'waist_L';
robot(14).collision(14).b    = [-0.05;-0.18;-0.123];
robot(14).collision(14).p    = [0;0;0];

robot(14).collision(15).name = 'waist_R';
robot(14).collision(15).b    = [0.05;-0.18;-0.123];
robot(14).collision(15).p    = [0;0;0];