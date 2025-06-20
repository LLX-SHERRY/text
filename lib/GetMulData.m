data_size = size(mJOINT);
H = data_size(1);
L = data_size(2);

m_state = mJOINT(:,1);
m_joint = mJOINT(:,2:L);

joint1 = [];
joint2 = [];

for i=1:1:H
    %1
    if(m_state(i) >0 && m_state(i)<=8)
        joint1 = [joint1;m_joint(i,:)];
    end
    
    %2
    if(m_state(i) >8 && m_state(i)<=12)
        joint2 = [joint2;m_joint(i,:)];
    end
end

GetRTXData(joint1,'data/crawl_down_temp.dat');
GetRTXData(joint2,'data/crawl_up_temp_2.dat');