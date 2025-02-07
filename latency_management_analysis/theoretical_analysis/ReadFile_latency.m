function [keys, values, ins_nums] = ReadFile_latency(filename)

fid = fopen(filename, 'r');
data = textscan(fid, '%s%f%f', 10000);
fclose(fid);

keys = data{1,1};
values = data{1,2};
ins_nums = data{1,3};

end