function [joint] = GetJointFromRTX(filename)
% 从RTX文件获取Crawl关节角度
    data = load(filename);
    h = length(data);
    %minus 
    data(:,4) = -data(:,4);
    data(:,5) = -data(:,5);
    data(:,6) = -data(:,6);
    data(:,11) = -data(:,11);
    data(:,12) = -data(:,12);
    data(:,13) = -data(:,13);
    data(:,17) = -data(:,17);
    data(:,21) = -data(:,21);
    data(:,24) = -data(:,24);
    data(:,26) = -data(:,26);
    data(:,29) = -data(:,29);
    joint = zeros(h,21);
    
    for i = 1:1:h
        %load leg
        for j = 0:1:5
            %load right leg
            joint(i,j+13) = data(i,j+2);
            %load left leg
            joint(i,j+4) = data(i,j+9);
        end
        
        %load right arm
            joint(i,10) = data(i,21);
            joint(i,11) = data(i,22);
            joint(i,12) = data(i,24);
            
        %load left arm
            joint(i,1) = data(i,26);
            joint(i,2) = data(i,27);
            joint(i,3) = data(i,29);
            
        %load waist
            joint(i,19) = data(i,16);
            joint(i,20) = data(i,17);
            joint(i,21) = data(i,18);
    end
end

