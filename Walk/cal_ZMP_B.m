function ZMP_B=cal_ZMP_B(pR,pL,RR,RL,FR,FL,taoR,taoL)
ZMP_R=cal_ZMP(FR,taoR);
ZMP_L=cal_ZMP(FL,taoL);

ZMP_R_B=pR+RR*[ZMP_R;0];
ZMP_L_B=pL+RL*[ZMP_L;0];
ZMP_R_B(3)=[];
ZMP_L_B(3)=[];

ZMP_B=(ZMP_R_B*FR(3)+ZMP_L_B*FL(3))/(FR(3)+FL(3));
