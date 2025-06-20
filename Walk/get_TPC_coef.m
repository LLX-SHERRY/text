function TPC_coef=get_TPC_coef(walk_para)
Tr=walk_para.Tr;
zc=walk_para.zc;
g=walk_para.g;
Ts=walk_para.Ts;


%% TPC Control
%连续状态空间表达式
Ac = [-1/Tr 1/Tr 0;
    0     0   1;
    0     0   0 ];
Bc = [-zc/(g*Tr); 0; 1];
Cc = [1 0 0];
Dc = 0;
%构建连续系统
sys = ss(Ac, Bc, Cc, Dc);
%系统离散化处理
dsys = c2d(sys, Ts);

%权重矩阵
Q_TPC = diag( [1 1 1] );
% R_TPC = 10^-4;
R_TPC = 10^-4;
%离散系统状态反馈控制系数计算
TPC_coef = -dlqr( dsys.A, dsys.B, Q_TPC, R_TPC);
