function [ file_name ] = SaveFile( dat,file_name )
%SAVEFILE Summary of this function goes here
%   Detailed explanation goes here
fp=fopen(file_name,'wt');
s=size(dat);
m=s(1);
n=s(2);
for i=1:1:m
    for j=1:1:n
        fprintf(fp,'%f\t',dat(i,j));
    end
    fprintf(fp,'\n');
end
fclose(fp);
end
