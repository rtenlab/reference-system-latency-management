function [keys, values] = ReadFile_exec(filename)

fid = fopen(filename, 'r');
data = textscan(fid, '%s%f', 10000);
fclose(fid);

keys = data{1,1};
values = data{1,2};

end