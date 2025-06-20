function Ek=calculate_Ek(robot,j)
if j==0
	Ek=0;
else
	c1=robot(j).R*robot(j).c;
	vc=robot(j).v+cross(robot(j).w,c1);	
	Ek=1/2*vc'*robot(j).m*vc+1/2*robot(j).w'*robot(j).R*robot(j).I*robot(j).R'*robot(j).w;
	Ek=Ek+calculate_Ek(robot,robot(j).sister)+calculate_Ek(robot,robot(j).child);
end

end