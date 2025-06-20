clear;clc;close;
robot=robot3D_description();
for ii=1:length(robot)
   robot(ii).q = 0.5; 
end
robot=fkinematic(robot,1);