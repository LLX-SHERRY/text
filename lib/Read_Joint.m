function [ joint ] = Read_Joint( data_path )
%READ_JOINT ��RTX�ļ�ת����VREP�ؽڽǶ�
%   Detailed explanation goes here
    data = load(data_path);
    joint = [data(:,2:7) data(:,9:14) data(:,16:18) data(:,21:22) data(:,24) data(:,26:27) data(:,29)];
end