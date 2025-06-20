close all;clear;clc;
syms q [23,1] positive
robot=robot3D_description();
robot(1).p=[0 0 0]';
robot(1).R=eye(3);

for ii=1:length(robot)-1
    robot(ii+1).q=q(ii);
%     robot(ii+1).dq=dq(ii);
end
robot=fkinematic(robot,1);
robot=fk_collision(robot);
