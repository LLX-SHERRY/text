function [file,h] = GetRTXData(joint, file_name)
    % 从Crawl关节角度生成RTX文件
    file = file_name;
    file_size = size(joint);
    h = file_size(1);
    
    data = zeros(h+5,30);
    
    for i = 1:1:h
        %load leg
        for j = 0:1:5
            %load right leg
            data(i,j+2) = joint(i,j+13);
            %load left leg
            data(i,j+9) = joint(i,j+4);
        end
        
        %load right arm
            data(i,21) = joint(i,10);
            data(i,22) = joint(i,11);
            data(i,24) = joint(i,12);
            
        %load left arm
            data(i,26) = joint(i,1);
            data(i,27) = joint(i,2);
            data(i,29) = joint(i,3);
            
        %load waist
            data(i,16) = joint(i,19);
            data(i,17) = joint(i,20);
            data(i,18) = joint(i,21);
    end
    
    data(h+1,:) = data(h,:);
    data(h+2,:) = data(h,:);
    data(h+3,:) = data(h,:);
    data(h+4,:) = data(h,:);
    data(h+5,:) = data(h,:);
    
    
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

    %save file
    SaveFile(data,file_name);
end
