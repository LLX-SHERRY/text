classdef MatlabVrep
    %MATLABVREP Summary of this class goes here
    %   Detailed explanation goes here
    %   Data Sequence 0 RLEG1-6 0 LLEG1-6 WZ_YAW WY_P WX_R 0 0 RARM1-4 0 LARM1-4
    properties
        Joint_Num;
        Joint_Name;
        Joint_Handle;
        Joint;
        Port;
        ClientID;
        Control_Time;
        Main;
        
        Body_Name;
        Body_Handle;
        UpBody_Name;
        UpBody_Handle;
        Thigh_Name;
        Thigh_Handle;
        Shank_Name;
        Shank_Handle;
        
        Force_sensor_Num;
        Force_sensor_Name;
        Force_sensor_Handle;
        
        CoM;
        CoM_Name;
        CoM_Handle;
    end
    
    methods
        function vrep = MatlabVrep(control_time)
            vrep.Joint_Num = 23;
            vrep.Control_Time = control_time;
            vrep.Joint_Name = cell(1,vrep.Joint_Num);
            vrep.Joint_Name = {...
                {'rightleg_joint1'},{'rightleg_joint2'},{'rightleg_joint3'},{'rightleg_joint4'},{'rightleg_joint5'},{'rightleg_joint6'},...%右腿
                {'leftleg_joint1'},{'leftleg_joint2'},{'leftleg_joint3'},{'leftleg_joint4'},{'leftleg_joint5'},{'leftleg_joint6'},...%左腿
                {'waist_jointz'},{'waist_jointx'},{'waist_jointy'},...%腰
                {'rightshoulder_joint1'},{'rightshoulder_joint2'},{'rightshoulder_joint3'},{'rightelbow_joint'},...%右手
                {'leftshoulder_joint1'},{'leftshoulder_joint2'},{'leftshoulder_joint3'},{'leftelbow_joint'}...%左手  主要这个一样 就可以和vrep通信ll
                };
            vrep.Joint = zeros(1,vrep.Joint_Num);
            vrep.Joint_Handle = zeros(1,vrep.Joint_Num);
            
            vrep.Body_Name = 'hip_d';
            vrep.Body_Handle = 16;
            vrep.UpBody_Name='Cuboid5';
            
%             vrep.Thigh_Name = 'leftthigh_d0';
%             vrep.Thigh_Handle = 99;
%             
%             vrep.Shank_Name = 'Cuboid0';
%             vrep.Shank_Handle = 109;

            vrep.Thigh_Name = 'rightthigh_d1';
            vrep.Thigh_Handle = 101;
            
            vrep.Shank_Name = 'Cuboid3';
            vrep.Shank_Handle = 110;
            
            vrep.Force_sensor_Num=2;
            vrep.Force_sensor_Name=cell(1,vrep.Force_sensor_Num);
            vrep.Force_sensor_Name={{'right_Force_sensor'},{'left_Force_sensor'}};
            
            vrep.Port = 19997;
            vrep.Main = remApi('remoteApi');
            
            vrep.CoM_Name = 'centerOfMassVisualizer';
            vrep.CoM_Handle = 0;
            
        end
        
        function mvrep = init(vrep)
            mvrep = vrep;
            mvrep.Main.simxFinish(-1);
            mvrep.ClientID = mvrep.Main.simxStart('127.0.0.1',mvrep.Port,true,true,5000,mvrep.Control_Time);
            fprintf('ClientID =  %d\n',mvrep.ClientID);
            for i =1:1:mvrep.Joint_Num
                joint_name = mvrep.Joint_Name{i};
                if i>0
                    joint_name = joint_name{1};
                end
                [~,mvrep.Joint_Handle(i)] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,joint_name,mvrep.Main.simx_opmode_oneshot_wait);
                
                mvrep.set_pid(i,2000.0,0.875,0.0000,0.00);
                
                mvrep.Joint = deg2rad([ 0 0 9.85  -20.08  10.23  0,...%右腿
                    0 0 9.85  -20.08  10.23  0,...%左腿
                    0,0,0,...%腰
                    0,0,0,0,...%右手
                    0,0,0,0%左手
                    ]);
                mvrep.set_joint();
                fprintf('Joint Handle %d = %d\n',i,mvrep.Joint_Handle(i));
            end
            mvrep.set_pid(14,999999,1,0.0000,0.00);
            % mvrep.set_pid(14,500,1,0.0000,0.00);
            mvrep.get_body_p();
            
            [~,mvrep.Body_Handle] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,mvrep.Body_Name,mvrep.Main.simx_opmode_oneshot_wait);
            fprintf('body Handle = %d\n',mvrep.Body_Handle);
            [~,mvrep.UpBody_Handle] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,mvrep.UpBody_Name,mvrep.Main.simx_opmode_oneshot_wait);
            fprintf('Upbody Handle = %d\n',mvrep.UpBody_Handle);
            [~,mvrep.Thigh_Handle] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,mvrep.Thigh_Name,mvrep.Main.simx_opmode_oneshot_wait);
            fprintf('Thigh Handle = %d\n',mvrep.Thigh_Handle);
            [~,mvrep.Shank_Handle] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,mvrep.Shank_Name,mvrep.Main.simx_opmode_oneshot_wait);
            fprintf('Shank Handle = %d\n',mvrep.Shank_Handle);
            
            
            for ii=1:mvrep.Force_sensor_Num
                force_sensor_name = mvrep.Force_sensor_Name{ii}{1};
                [~,mvrep.Force_sensor_Handle(ii)] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,force_sensor_name,mvrep.Main.simx_opmode_oneshot_wait);
                fprintf('Force sensor Handle = %d\n',mvrep.Force_sensor_Handle(ii));
            end
            [~,mvrep.CoM_Handle] = mvrep.Main.simxGetObjectHandle(mvrep.ClientID,mvrep.CoM_Name,mvrep.Main.simx_opmode_oneshot_wait);
            fprintf('CoM Handle = %d\n',mvrep.CoM_Handle);
        end
        
        function [] = go(vrep)
            vrep.Main.simxSynchronous(vrep.ClientID,true);
            vrep.Main.simxStartSimulation(vrep.ClientID,vrep.Main.simx_opmode_blocking);
            vrep.trigger();
        end
        
        function [] = trigger(vrep)
            vrep.Main.simxSynchronousTrigger(vrep.ClientID);
        end
        
        function [] = pause(vrep)
            vrep.Main.simxPauseSimulation(vrep.ClientID,vrep.Main.simx_opmode_blocking);
        end
        
        function [] = stop(vrep)
            % stop the simulation:
            vrep.Main.simxStopSimulation(vrep.ClientID,vrep.Main.simx_opmode_blocking);
            % Now close the connection to V-REP:
            vrep.Main.simxFinish(vrep.ClientID);
        end
        
        function [] = set_joint(vrep)
            for i=1:1:vrep.Joint_Num
                vrep.Main.simxSetJointTargetPosition(vrep.ClientID,vrep.Joint_Handle(i),vrep.Joint(i),vrep.Main.simx_opmode_oneshot);
            end
        end
        
        function [] = set_joint_initial(vrep,joint_initial)
            for i=1:1:vrep.Joint_Num
                vrep.Main.simxSetJointPosition(vrep.ClientID,vrep.Joint_Handle(i),joint_initial(i),vrep.Main.simx_opmode_oneshot);
            end
        end
        
        function [] = keep_time(vrep)
            vrep.Main.simxGetPingTime(vrep.ClientID)
        end
        
        function [] = set_pid(vrep,joint_num,torque_limit,p,i,d)
            vrep.Main.simxSetJointForce(vrep.ClientID,vrep.Joint_Handle(joint_num),torque_limit,vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetObjectFloatParameter(vrep.ClientID,vrep.Joint_Handle(joint_num),2002,p,vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetObjectFloatParameter(vrep.ClientID,vrep.Joint_Handle(joint_num),2003,i,vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetObjectFloatParameter(vrep.ClientID,vrep.Joint_Handle(joint_num),2004,d,vrep.Main.simx_opmode_oneshot);
        end
        
        function [] = set_body_o(vrep,eulerAngles)
            vrep.Main.simxSetObjectOrientation(vrep.ClientID, vrep.Body_Handle, -1,eulerAngles,vrep.Main.simx_opmode_oneshot);
        end
        
        function [] = set_body_p(vrep,position)
            vrep.Main.simxSetObjectPosition(vrep.ClientID, vrep.Body_Handle, -1,position,vrep.Main.simx_opmode_oneshot);
        end
            
        function [body_v,ang_v] = get_body_v(vrep)
            [~,body_v,ang_v]= vrep.Main.simxGetObjectVelocity(vrep.ClientID, vrep.Body_Handle, vrep.Main.simx_opmode_oneshot);
        end
        
        function body_p = get_body_p(vrep)
            [~,body_p]= vrep.Main.simxGetObjectPosition(vrep.ClientID, vrep.Body_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end
        
        function body_eul=get_body_eul(vrep)
            [~,body_eul] = vrep.Main.simxGetObjectOrientation(vrep.ClientID, vrep.Body_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end
        
        function [thigh_v,thigh_w] = get_thigh_v(vrep)
            [~,thigh_v,thigh_w]= vrep.Main.simxGetObjectVelocity(vrep.ClientID, vrep.Thigh_Handle, vrep.Main.simx_opmode_oneshot);
        end
        
        function thigh_p = get_thigh_p(vrep)
            [~,thigh_p]= vrep.Main.simxGetObjectPosition(vrep.ClientID, vrep.Thigh_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end
        
        function thigh_eul=get_thigh_eul(vrep)
            [~,thigh_eul] = vrep.Main.simxGetObjectOrientation(vrep.ClientID, vrep.Thigh_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end 
        
        function [shank_v,shank_w] = get_shank_v(vrep)
            [~,shank_v,shank_w]= vrep.Main.simxGetObjectVelocity(vrep.ClientID, vrep.Shank_Handle, vrep.Main.simx_opmode_oneshot);
        end
        
        function shank_p = get_shank_p(vrep)
            [~,shank_p]= vrep.Main.simxGetObjectPosition(vrep.ClientID, vrep.Shank_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end
        
        function shank_eul=get_shank_eul(vrep)
            [~,shank_eul] = vrep.Main.simxGetObjectOrientation(vrep.ClientID, vrep.Shank_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end 
        
        function q_rel = get_Joint_position(vrep)
            q_rel = zeros(1,vrep.Joint_Num);
            for i=1:1:vrep.Joint_Num
                [~,position] = vrep.Main.simxGetJointPosition(vrep.ClientID,vrep.Joint_Handle(i),vrep.Main.simx_opmode_oneshot);
                q_rel(i) = position;
            end
        end
        
        function [F,tao]=get_force_sensor(vrep)
            F=zeros(2,3);
            tao=zeros(2,3);
            for ii=1:vrep.Force_sensor_Num
                [~,~,forceVector,torqueVector]= vrep.Main.simxReadForceSensor(vrep.ClientID,vrep.Force_sensor_Handle(ii),vrep.Main.simx_opmode_oneshot);
                F(ii,:)=forceVector;
                tao(ii,:)=torqueVector;
            end
        end
        
        function p_CoM=get_CoM_position(vrep)
            [~,p_CoM]= vrep.Main.simxGetObjectPosition(vrep.ClientID, vrep.CoM_Handle, -1, vrep.Main.simx_opmode_oneshot);
        end
        
        function [] = Push(vrep,forceX,forceY,forceZ)
            [~,ore] = vrep.Main.simxGetObjectOrientation(vrep.ClientID,vrep.UpBody_Handle,-1,vrep.Main.simx_opmode_oneshot);
            R = eul2rotm(ore,'xyz');
            wF = [forceX,forceY,forceZ]';
            bF = R'*wF;
            
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceX',bF(1),vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceY',bF(2),vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceZ',bF(3),vrep.Main.simx_opmode_oneshot);
        end
        
        function [] = Stop_Push(vrep)
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceX',0,vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceY',0,vrep.Main.simx_opmode_oneshot);
            vrep.Main.simxSetFloatSignal(vrep.ClientID,'forceZ',0,vrep.Main.simx_opmode_oneshot);
        end
        
        function f=get_Joint_force(vrep)
            f = zeros(1,vrep.Joint_Num);
            for i=1:1:vrep.Joint_Num
                [~,force] = vrep.Main.simxGetJointForce(vrep.ClientID,vrep.Joint_Handle(i),vrep.Main.simx_opmode_oneshot);
                f(i) = force;
            end
        end
    end
    
end

