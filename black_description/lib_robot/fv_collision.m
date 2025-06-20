function robot=fv_collision(robot)

for jj=1:length(robot)
    
    for ii=1:length(robot(jj).collision)
        robot(jj).collision(ii).v=robot(jj).v+hat3(robot(jj).w)*robot(jj).R* robot(jj).collision(ii).b;        
		robot(jj).collision(ii).w=robot(jj).w;        
    end
    
end
end