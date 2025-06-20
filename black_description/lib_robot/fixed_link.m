function [fixed_object,object]=fixed_link(robot,fixed_id,fixed_frame)
m=0;
mc=[0 0 0]';
II=zeros(3,3);
for ii=1:length(fixed_id)
	m=robot(fixed_id(ii)).m+m;
	pp=fixed_frame.R'*(robot(fixed_id(ii)).p-fixed_frame.p);
	object(ii).m=robot(fixed_id(ii)).m;
	object(ii).RR=fixed_frame.R'*robot(fixed_id(ii)).R;
	object(ii).pc=pp+object(ii).RR*robot(fixed_id(ii)).c;
	object(ii).II=object(ii).RR*robot(fixed_id(ii)).I*object(ii).RR';
	mc=robot(fixed_id(ii)).m*object(ii).pc+mc;
end
c=mc/m;

for ii=1:length(fixed_id)
	object(ii).pc_com=object(ii).pc-c;
	II=II+object(ii).II+object(ii).m*hat3(object(ii).pc_com)*hat3(object(ii).pc_com)';
end
fixed_object.m=m;
fixed_object.c=c;
fixed_object.I=II;
end