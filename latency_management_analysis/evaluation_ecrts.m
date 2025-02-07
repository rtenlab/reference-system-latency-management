%% Run this to clear everything
clear all;
close all;
clc;
%% Run this section to parse the data from a run with the latency management executor
% Define chainStruct template
chainStruct = struct(...
    'chain_id', [], ...
    'priority', [], ...
    'instance', [], ...
    'response_time', [] ...
);

boundStruct = struct(...
    'chain_id', [], ...
    'wcrt', [] ...
    );

% Import latency management data
%latency_mgmt_data = importfile1("latency_mgmt_output.txt");
latency_mgmt_data = importfile1("analysis_testing13");
chain_response_times = [];

% Initialize lists for best-effort (BE) and real-time (RT) chain response times
be_chain_response_times = [];
rt_chain_response_times = [];
be_chain_bounds = [];
% Parallelize for faster execution, this is taking way too long for the
% processing to actually happen.
% 
parfor i = 1:height(latency_mgmt_data)
    % Extract the value from VarName3 to compare with "prio"
    test_col = string(latency_mgmt_data.VarName4(i));
    
    % Check if the value is equal to "prio"
    if strcmp(test_col, "prio")
        % Create a new instance of the chainStruct with appropriate values
        valid_row = struct(...
            'chain_id', latency_mgmt_data.VarName3(i), ...
            'priority', latency_mgmt_data.VarName5(i), ...
            'instance', latency_mgmt_data.VarName7(i), ...
            'response_time', latency_mgmt_data.VarName10(i) ...
            );
        
        % Append to the chain_response_times array
        if strcmp(string(latency_mgmt_data.Thread(i)), "RT")
            rt_chain_response_times = [rt_chain_response_times; valid_row];
        elseif strcmp(string(latency_mgmt_data.Thread(i)), "BE")
            be_chain_response_times = [be_chain_response_times; valid_row];
            
        end
    elseif strcmp(test_col, "WCRT")
        bound_row = struct(...
            'chain_id', latency_mgmt_data.VarName3(i), ...
            'wcrt', latency_mgmt_data.VarName5(i) ...
            );
        be_chain_bounds = [be_chain_bounds; bound_row];
    end
end



% filter data errors
% Convert to table
tempTable = struct2table(rt_chain_response_times);
tempTable = rmmissing(tempTable);
rt_chain_response_times = table2struct(tempTable);


tempTable = struct2table(be_chain_response_times);
tempTable = rmmissing(tempTable);
be_chain_response_times = table2struct(tempTable);


tempTable = struct2table(be_chain_bounds);
tempTable = rmmissing(tempTable);
be_chain_bounds= table2struct(tempTable);


% Extract unique RT chain IDs from rt_chain_response_times
rt_chain_ids = unique([rt_chain_response_times.chain_id]);

% Collect response times grouped by each chain ID
response_times_grouped = cell(length(rt_chain_ids), 1);
parfor k = 1:length(rt_chain_ids)
    % Extract data for the current chain ID
    chain_id = rt_chain_ids(k);
    chain_data = rt_chain_response_times([rt_chain_response_times.chain_id] == chain_id);
    
    % Collect response times
    response_times_grouped{k} = [chain_data.response_time];
end

% Create a figure for the RT chains boxplot
figure;
boxplot(cell2mat(response_times_grouped'), repelem(rt_chain_ids, cellfun(@length, response_times_grouped)), 'Labels', arrayfun(@(x) sprintf('RT Chain %d', x), rt_chain_ids, 'UniformOutput', false));
title('Response Times for RT Chains (LaME Data)');
xlabel('Chain ID');
ylabel('Response Time (\mus)');
ylim([0, 2.5e5]);
grid minor;
rt_deadlines = [200000 200000 100000];
xt = [0.75 1.75 2.75];
yt = rt_deadlines + 10000;
str = ["D = 200ms" "D = 200ms" "D = 100ms"];
line([0.5 1.5],rt_deadlines(1)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([1.5 2.5],rt_deadlines(2)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([2.5 3.5],rt_deadlines(3)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
text(xt, yt, str);
% Extract unique BE chain IDs from be_chain_response_times
be_chain_ids = unique([be_chain_response_times.chain_id]);

% Collect response times grouped by each chain ID
response_times_grouped_be = cell(length(be_chain_ids), 1);
bounds_grouped_be = cell(length(be_chain_ids), 1);
for k = 1:length(be_chain_ids)
    % Extract data for the current chain ID
    chain_id = be_chain_ids(k);
    chain_data = be_chain_response_times([be_chain_response_times.chain_id] == chain_id);
    chain_bound = be_chain_bounds([be_chain_bounds.chain_id] == chain_id);
    % Collect response times
    response_times_grouped_be{k} = [chain_data.response_time];
    bounds_grouped_be{k} = [chain_bound.wcrt];
end

bounds = [];
for i = 1:length(bounds_grouped_be)
    bounds = [bounds, bounds_grouped_be{i,1}(end)];
end

% Create a figure for the BE chains boxplot
figure;
boxplot(cell2mat(response_times_grouped_be'), repelem(be_chain_ids, cellfun(@length, response_times_grouped_be)), 'Labels', arrayfun(@(x) sprintf('BE Chain %d', x), be_chain_ids, 'UniformOutput', false));
title('Response Times for BE Chains (LaME Data)');
xlabel('Chain ID');
ylabel('Response Time (\mus)');
ylim([0, 2.5e5]);
grid minor;
x =[];
y=[];
for i = 1:length(bounds)
    %x = [x; [i-0.5 0.5+i]];
    %y = [y; bounds(i)*[1 1]];
    line([i-0.5 i+0.5], bounds(i)*[1 1], 'Color','magenta', 'LineWidth', 1.5)
    text(i-0.25, bounds(i) + 5000, sprintf("B = %.0f ms", bounds(i)/1000))
end    





%% Run this section to parse and graph the results from the multi-threaded PiCAS test

picas_data = importfile1("picas.txt");
picas_chain_response_times = [];
% Initialize lists for best-effort (BE) and real-time (RT) chain response times
be_picas_chain_response_times = [];
rt_picas_chain_response_times = [];


for i = 1:height(picas_data)
    % Extract the value from VarName3 to compare with "prio"
    test_col = string(picas_data.VarName4(i));
    
    % Check if the value is equal to "prio"
    if strcmp(test_col, "prio")
        % Create a new instance of the chainStruct with appropriate values
        valid_row = struct(...
            'chain_id', picas_data.VarName3(i), ...
            'priority', picas_data.VarName5(i), ...
            'instance', picas_data.VarName7(i), ...
            'response_time', picas_data.VarName10(i) ...
            );
        
        % Append to the chain_response_times array
        if strcmp(string(picas_data.Thread(i)), "RT")
            rt_picas_chain_response_times = [rt_picas_chain_response_times; valid_row];
        elseif strcmp(string(picas_data.Thread(i)), "BE")
            be_picas_chain_response_times = [be_picas_chain_response_times; valid_row];
        end
        end
end

% Convert to table
tempTable = struct2table(rt_picas_chain_response_times);
tempTable = rmmissing(tempTable);
rt_picas_chain_response_times = table2struct(tempTable);

tempTable = struct2table(be_picas_chain_response_times);
tempTable = rmmissing(tempTable);
be_picas_chain_response_times = table2struct(tempTable);

% Extract unique RT chain IDs from rt_picas_chain_response_times
rt_picas_chain_ids = unique([rt_picas_chain_response_times.chain_id]);
be_picas_chain_ids = unique([be_picas_chain_response_times.chain_id]);

% Collect response times grouped by each chain ID
response_times_grouped_rt_picas = cell(length(rt_picas_chain_ids), 1);
response_times_grouped_be_picas = cell(length(be_picas_chain_ids), 1);

parfor k = 1:length(rt_picas_chain_ids)
    % Extract data for the current chain ID
    chain_id = rt_picas_chain_ids(k);
    chain_data = rt_picas_chain_response_times([rt_picas_chain_response_times.chain_id] == chain_id);
    
    % Collect response times
    response_times_grouped_rt_picas{k} = [chain_data.response_time];
end

% Create a figure for the RT chains boxplot
figure;
boxplot(cell2mat(response_times_grouped_rt_picas'), repelem(rt_picas_chain_ids, cellfun(@length, response_times_grouped_rt_picas)), 'Labels', arrayfun(@(x) sprintf('RT Chain %d', x), rt_picas_chain_ids, 'UniformOutput', false));
title('Response Times for RT Chains (PICAS Data)');
xlabel('Chain ID');
ylabel('Response Time (\mus)');
ylim([0, 2.5e5]);
grid minor;
line([0.5 1.5],rt_deadlines(1)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([1.5 2.5],rt_deadlines(2)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([2.5 3.5],rt_deadlines(3)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
text(xt, yt, str);

parfor k = 1:length(be_picas_chain_ids)
    % Extract data for the current chain ID
    chain_id = be_picas_chain_ids(k);
    chain_data = be_picas_chain_response_times([be_picas_chain_response_times.chain_id] == chain_id);
    
    % Collect response times
    response_times_grouped_be_picas{k} = [chain_data.response_time];
end

% Create a figure for the BE chains boxplot
figure;
boxplot(cell2mat(response_times_grouped_be_picas'), repelem(be_picas_chain_ids, cellfun(@length, response_times_grouped_be_picas)), 'Labels', arrayfun(@(x) sprintf('BE Chain %d', x), be_picas_chain_ids, 'UniformOutput', false));
title('Response Times for BE Chains (PICAS Data)');
xlabel('Chain ID');
ylabel('Response Time (\mus)');
ylim([0, 2.5e5]);
grid minor;

%% Run this to parse and graph the results from a run with the default ROS 2 executor

% Import default latency management data
default_data = importfile1("default.txt");

% Initialize chain response times
% Initialize lists for best-effort (BE) and real-time (RT) chain response times
be_default_chain_response_times = [];
rt_default_chain_response_times = [];

% Loop through default data and extract valid rows
for i = 1:height(default_data)
    % Extract the value from VarName3 to compare with "prio"
    test_col = string(default_data.VarName4(i));
    
    % Check if the value is equal to "prio"
    if strcmp(test_col, "prio")
        % Create a new instance of the chainStruct with appropriate values
        valid_row = struct(...
            'chain_id', default_data.VarName3(i), ...
            'priority', default_data.VarName5(i), ...
            'instance', default_data.VarName7(i), ...
            'response_time', default_data.VarName10(i) ...
            );
        
        % Append to the chain_response_times array
        if strcmp(string(default_data.Thread(i)), "RT")
            rt_default_chain_response_times = [rt_default_chain_response_times; valid_row];
        elseif strcmp(string(default_data.Thread(i)), "BE")
            be_default_chain_response_times = [be_default_chain_response_times; valid_row];
        end
        end
end
% filter data errors
% Convert to table
tempTable = struct2table(rt_default_chain_response_times);
tempTable = rmmissing(tempTable);
rt_default_chain_response_times = table2struct(tempTable);

tempTable = struct2table(be_default_chain_response_times);
tempTable = rmmissing(tempTable);
be_default_chain_response_times = table2struct(tempTable);


% Analysis and Graphing for Real-Time Chains in Default Data
% Extract chain IDs for RT chains
rt_default_chain_ids = unique([rt_default_chain_response_times.chain_id]);
be_default_chain_ids = unique([be_default_chain_response_times.chain_id]);

% Collect response times grouped by each chain ID
response_times_grouped_rt_default = cell(length(rt_default_chain_ids), 1);
response_times_grouped_be_default = cell(length(be_default_chain_ids), 1);
% Prepare data for RT chains boxplot
rt_default_response_times_data = [];
rt_group_labels = [];

parfor k = 1:length(rt_default_chain_ids)
    % Extract data for the current chain ID
    chain_id = rt_default_chain_ids(k);
    chain_data = rt_default_chain_response_times([rt_default_chain_response_times.chain_id] == chain_id);
    
    % Collect response times
    response_times_grouped_rt_default{k} = [chain_data.response_time];
end

% Create a boxplot for RT chains
figure;
boxplot(cell2mat(response_times_grouped_rt_default'), repelem(rt_default_chain_ids, cellfun(@length, response_times_grouped_rt_default)), 'Labels', arrayfun(@(x) sprintf('RT Chain %d', x), rt_default_chain_ids, 'UniformOutput', false));
%boxplot(rt_default_response_times_data, rt_group_labels, 'Colors', 'b');
grid minor;
ylabel('Response Time (\mus)');
xlabel('Real-Time Chains');
ylim([0, 2.5e5]);
title('Response Times for RT Chains (Default Data)');
line([0.5 1.5],rt_deadlines(1)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([1.5 2.5],rt_deadlines(2)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
line([2.5 3.5],rt_deadlines(3)*([1 1]),'Color','magenta', 'LineWidth',1.5 );
text(xt, yt, str);
hold off;

% Analysis and Graphing for Best-Effort Chains in Default Data
% Extract chain IDs for BE chains

% Prepare data for BE chains boxplot
be_default_response_times_data = [];
be_group_labels = [];

parfor k = 1:length(be_default_chain_ids)
    % Extract data for the current chain ID
    chain_id = be_default_chain_ids(k);
    chain_data = be_default_chain_response_times([be_default_chain_response_times.chain_id] == chain_id);
    
    % Collect response times
    response_times_grouped_be_default{k} = [chain_data.response_time];
end

% Create a boxplot for BE chains
figure;
boxplot(cell2mat(response_times_grouped_be_default'), repelem(be_default_chain_ids, cellfun(@length, response_times_grouped_be_default)), 'Labels', arrayfun(@(x) sprintf('BE Chain %d', x), be_default_chain_ids, 'UniformOutput', false));
%boxplot(be_default_response_times_data, be_group_labels, 'Colors', 'r');
ylabel('Response Time (\mus)');
xlabel('Best-Effort Chains');
title('Response Times for BE Chains (Default Data)');
grid minor;
ylim([0, 2.5e5]);
hold off;




