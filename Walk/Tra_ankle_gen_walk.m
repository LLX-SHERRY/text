function Tra_ankle=Tra_ankle_gen_walk(robot,walk_para)

%% 初始化
L_step=walk_para.L_step;
N_step=walk_para.N_step;
T_step=walk_para.T_step;
T_single=T_step*(1-walk_para.P_double);
T_double=T_step*walk_para.P_double;

T_ready=walk_para.T_ready;
T_total=2*T_ready+N_step*T_step;

Ts=walk_para.Ts;
Nk_total=round(T_total/Ts)+1;
Nk_ready=round(T_ready/Ts);
Nk_1T=round(T_step/Ts);
Nk_double=round(T_double/Ts);

Ankle_width=-robot(14).p(1)+robot(7).p(1)+0.0;
H_step=walk_para.H_step;
%% x方向
%左脚
xL=-0.5*Ankle_width*ones(1,Nk_total);
%右脚
xR=0.5*Ankle_width*ones(1,Nk_total);
%% y方向
y_1T=TSpline_S_V_A(0,0,0,0,L_step,T_single/2,2*L_step,0,0,T_single,Ts);
y_1T=[y_1T(1,1:end-1),2*L_step*ones(1,Nk_double)];
y_ready=TSpline_S_V_A(0,0,0,0,L_step/2,T_single/2,L_step,0,0,T_single,Ts);
y_ready=y_ready(1,1:end-1);
yL=[];
yR=[];
for ii=1:1:N_step
    if ii==1
        %左脚
        yL_temp=[0*ones(1,Nk_ready),y_ready,L_step*ones(1,Nk_double)];
        %右脚
        yR_temp=[0*ones(1,Nk_ready),0*ones(1,Nk_1T)];
    else
        if ii~=N_step
            if mod(ii,2)
                yR_temp=(ii-1)*L_step*ones(1,Nk_1T);
                yL_temp=y_1T+(ii-2)*L_step;
            else
                yR_temp=y_1T+(ii-2)*L_step;
                yL_temp=(ii-1)*L_step*ones(1,Nk_1T);
            end
        else
            if mod(ii,2)
                yR_temp=[(ii-1)*L_step*ones(1,Nk_1T),(ii-1)*L_step*ones(1,Nk_ready+1)];
                yL_temp=[y_ready+(ii-2)*L_step,(ii-1)*L_step*ones(1,Nk_double),(ii-1)*L_step*ones(1,Nk_ready+1)];
            else
                yR_temp=[y_ready+(ii-2)*L_step,(ii-1)*L_step*ones(1,Nk_double),(ii-1)*L_step*ones(1,Nk_ready+1)];
                yL_temp=[(ii-1)*L_step*ones(1,Nk_1T),(ii-1)*L_step*ones(1,Nk_ready+1)];                
            end
        end
    end
    yL=[yL,yL_temp];
    yR=[yR,yR_temp];
end

%% z方向
z_1T=TSpline_S_V_A(0,0,0,0,H_step,T_single/2,0,0,0,T_single,Ts);
z_1T=[z_1T(1,1:end-1),0*ones(1,Nk_double)];

zL=0*ones(1,Nk_ready);
zR=0*ones(1,Nk_ready);
for ii=1:1:N_step
    if mod(ii,2)
        zL_temp=z_1T;
        zR_temp=0*ones(1,Nk_1T);
    else
        zR_temp=z_1T;
        zL_temp=0*ones(1,Nk_1T);
    end
    zL=[zL,zL_temp];
    zR=[zR,zR_temp];
end
zL=[zL,0*ones(1,Nk_ready+1)];
zR=[zR,0*ones(1,Nk_ready+1)];

%% 
Tra_ankle.xL=xL;
Tra_ankle.xR=xR;
Tra_ankle.yL=yL;
Tra_ankle.yR=yR;
Tra_ankle.zL=zL;
Tra_ankle.zR=zR;