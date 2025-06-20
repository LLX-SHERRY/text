function robot=ik_collision(robot,Target,c_id)


% num_joint=length(robot)-1;
% lambda=0.005;
robot=fkinematic(robot,1);
robot=fk_collision(robot);
route=findroute(robot,c_id(1));

err_last=zeros(6,1);
for n=1:100
	J=calculate_jacobian(robot,route,c_id(2));
	rel.p=robot(c_id(1)).collision(c_id(2)).p;
	rel.R=robot(c_id(1)).R;
	
	err=calculate_err(Target,rel);
	if norm(err)<1e-6||norm(abs((err_last-err)/err))<1e-2
		robot=fkinematic(robot,1);
		robot=fk_collision(robot);
		return
	end
	err_last=err;
	if norm(err)>1
		lambda=0.5;
	else
		if norm(err)<=1&&norm(err)>1e-5
			lambda=0.1;
		else
			
			lambda=0.02;
		end
		
	end
	J_in=(J*J'+lambda^2*eye(6))^(-1);
	dq=J'*J_in*err;
	
	for nn=1:length(route)
		j=route(nn);
		robot(j).q=robot(j).q+dq(nn);
% 		robot(j).q=max(robot(j).q,robot(j).lb);
		robot(j).q=min(robot(j).q,robot(j).ub);
	end
	robot=fkinematic(robot,1);
	robot=fk_collision(robot);
end
end