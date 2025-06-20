global Out;
global bhr6;
if(bhr6.StateFlag ~=0)
Out = CrawlUpdate(bhr6.BasePosture,bhr6.JointAngle,bhr6.Inter_Time*1000);
end