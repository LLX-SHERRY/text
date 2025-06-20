function x=ik_OPT(robot,target,dqt_g)
if nargin==2
	dqt_g=1e-3;
end
for ii=1:length(robot)-1
	x0(ii,1)=robot(ii+1).q;
	lb(ii,1)=robot(ii+1).lb;
	ub(ii,1)=robot(ii+1).ub;
	
end
if norm(dqt_g)<1e-5
	dqt_g=1e-3;
end
step=1;
x_norm=x0+dqt_g*step;
options = optimset('MaxIter',250,'MaxFunEvals',10000,'TolX',1e-7,'TolFun',1e-7,'TolCon',1e-6,'Display','iter');
x=fmincon(@cost,x0,[],[],[],[],lb,ub,@tgt_follow,options,robot,target,x_norm);

end

function j=cost(q,~,~,x_norm)
j=norm(q-x_norm);
end

function [C,Ceq]=tgt_follow(q,robot,target,~)
C=[];
Ceq=[];
for ii=1:length(robot)-1
	robot(ii+1).q=q(ii);
end
robot=fkinematic(robot,1);
robot=fk_collision(robot);
for ii=1:length(target)
	link_num=target(ii).id(1);
	col_num=target(ii).id(2);
	
	Ceq=[Ceq;calculate_err(target(ii),robot(link_num).collision(col_num))];
end
end