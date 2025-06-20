function Ep=calculate_Ep(robot,j)
Mc=calculate_Mc(robot,j);
Ep=Mc(3)*9.8;
end