Data1 = load('crouch.dat');
Data2 = load('data/crawl_up_temp_2.dat');
Data_Size = size(Data1);
Data_Len = Data_Size(1);

Data1 = Data1(3500:Data_Len,:);

Data = [Data2;Data1];

SaveFile(Data,'data/crawl_up_temp.dat');
delete data/crawl_up_temp_2.dat