classdef NUM
    %NUM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %----NUM in u and state----%
        LEFT_ARM_1 = 1;
        LEFT_ARM_2 = 2;
        LEFT_ARM_3 = 3;

        LEFT_LEG_1 = 4;
        LEFT_LEG_2 = 5;
        LEFT_LEG_3 = 6;
        LEFT_LEG_4 = 7;
        LEFT_LEG_5 = 8;
        LEFT_LEG_6 = 9;

        RIGHT_ARM_1 = 10;
        RIGHT_ARM_2 = 11;
        RIGHT_ARM_3 = 12;

        RIGHT_LEG_1 = 13;
        RIGHT_LEG_2 = 14;
        RIGHT_LEG_3 = 15;
        RIGHT_LEG_4 = 16;
        RIGHT_LEG_5 = 17;
        RIGHT_LEG_6 = 18;
                
        BASE_RX = 22;
        BASE_RY = 23;
        BASE_RZ = 24;
        BASE_X = 25;
        BASE_Y = 26;
        BASE_Z = 27;
        %====NUM in u and state====%
        
        %----NUM in state----%
        WAIST_1 = 19;
        WAIST_2 = 20;
        WAIST_3 = 21;
        %====NUM state====%
        
        %----NUM in u----%
        M_WAIST_ROLL = 19;
        M_WAIST_PITCH = 20;
        M_WAIST_YAW = 21;        
        LEFT_FOOT_FORCE_Z_1 = 28
        LEFT_FOOT_FORCE_Z_2 = 29
        LEFT_FOOT_FORCE_Z_3 = 30
        LEFT_FOOT_FORCE_Z_4 = 31
        LEFT_HAND_FORCE_Z   = 32
        LEFT_HAND_TORQUE    = 33
        
        RIGHT_FOOT_FORCE_Z_1 = 34
        RIGHT_FOOT_FORCE_Z_2 = 35
        RIGHT_FOOT_FORCE_Z_3 = 36
        RIGHT_FOOT_FORCE_Z_4 = 37
        RIGHT_HAND_FORCE_Z   = 38
        RIGHT_HAND_TORQUE    = 39
        %====NUM in u====%

        %----NUM in sub item----%
        RX = 1; RY = 2; RZ = 3;
        X  = 4;  Y = 5;  Z = 6;
        
        LFZ1 = 1; RFZ1 = 7; 
        LFZ2 = 2; RFZ2 = 8;
        LFZ3 = 3; RFZ3 = 9;
        LFZ4 = 4; RFZ4 = 10;
        LHF  = 5; RHF  = 11;
        LHT  = 6; RHT  = 12;
        %====NUM in sub item====%

        
        OUT_FX = 22;
        OUT_FY = 23;
        OUT_FZ = 24;
        
        %----NUM for robot control----%
        ITEM_DEFAULT    =   00;
        ITEM_WAIST		=	01;
        ITEM_BASE		=	02;
        ITEM_WAIST_BASE	=	03;
        ITEM_LEFT_HAND	=	04;
        ITEM_LEFT_KNEE	=	05;
        ITEM_LEFT_FOOT	=	06;	
        ITEM_RIGHT_HAND	=	07;
        ITEM_RIGHT_KNEE	=	08;
        ITEM_RIGHT_FOOT	=	09;
        ITEM_DOUBLE_HANDS =	10;

        MOVE_DEFAULT	=	00;
        MOVE_RX_ADD		=	11;
        MOVE_RX_SUB		=	10;
        MOVE_RY_ADD		=	21;
        MOVE_RY_SUB		=	20;
        MOVE_RZ_ADD		=	31;
        MOVE_RZ_SUB		=	30;
        MOVE_X_ADD		=	41;
        MOVE_X_SUB		=	40;
        MOVE_Y_ADD		=	51;
        MOVE_Y_SUB		=	50;
        MOVE_Z_ADD		=	61;
        MOVE_Z_SUB		=	60;
        
        MOVE_SUB = 0;
        MOVE_ADD = 1;
        %====NUM for robot control====%
    end
    
    methods
    end
    
end

