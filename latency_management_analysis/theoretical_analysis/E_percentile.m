%% work for *_E
clc; clear all; close all;

data = importdata("data/First-CG/Normal/mt4/C1T_8_E.txt", " ");
T = struct2table(data);
sortedT = sortrows(T, "textdata");

col_name = []; data = struct();
for i = 1 : size(sortedT, 1)

    cell_name = table2cell(sortedT(i, "textdata"));
    cell_value = table2cell(sortedT(i, "data"));
    [~, idx] = find(strcmp(col_name, cell_name{1,1}));
    if isempty(idx)
        name = convertCharsToStrings(cell_name{1,1});
        col_name = [col_name, name];
        data.(name) = [];
        data.(name)(1) = cell_value{1,1}(1);
    else
        data.(col_name(idx))(end+1) = cell_value{1,1}(1); % for R measure
    end
%     if i == 10000
%         break;
%     end
end

result = [];
for i = 1 : size(col_name, 2)
    result = [result; col_name(i), prctile(data.(col_name(i)), 99) mean(data.(col_name(i))) max(data.(col_name(i))) min(data.(col_name(i)))];
end