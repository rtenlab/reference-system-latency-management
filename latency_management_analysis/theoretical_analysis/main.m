% test with txt file
clc; clear all; close all;

M = 5;  % number of processors
PRIO = 0;   % priority-driven flag, 1 : priority-driven, 0 : non priority
CG_enabled = 1;   % CG flag, 1 : Mutually-exclusive, 0 : Reentrant

% fid = fopen('num_chains_num_callbacks_data/10_chains.txt', 'r');
% fid = fopen('data/schedulability_test_CD/chains_cpu4_util_2_0.txt', 'r');
% fid = fopen('thread_1_AD.txt', 'r');
% fid = fopen('picas_casestudy2_less.txt', 'r');
fid = fopen('ARS_Config.txt', 'r');

% 0 - 1% for how much you are throttling 
speed_factor = 1;

%fid = fopen('ARS_Throttled.txt', 'r');

%fid = fopen('9_chains_AD.txt', 'r');
%fid = fopen('util_1.txt', 'r');
data = textscan(fid, '%f%f%f%d%d', 'Delimiter', '-');
fclose(fid);

data{1, 2}(:) = data{1, 2}(:) / speed_factor;
chainset = []; chain = []; resultset = [];
num_chain = 1;
for i = 1 : size(data{1, 1}, 1)
    if isnan(data{1, 1}(i))
        if ~isempty(chain)
            chainset = [chainset; chain];
        end

        % Find the response-time
%         [R, S, SCHED] = PWA_AD(chainset, M, PRIO,CG_enabled);
        [R, S, SCHED] = PWA_CD(chainset, M, PRIO, CG_enabled, 1000);
%          [R, S, SCHED] = Casini(chainset);


        P = []; C = [];
        for c = 1 : size(chainset, 1)
            P = [P; chainset(c).T];
            C = [C; sum(chainset(c).C)];
        end
        result = struct('chainset_id', num_chain, 'SCHED', SCHED, 'R', R, 'S', S, 'P', P, 'C', C);
        resultset = [resultset; result];

        num_chain = num_chain + 1;
        chainset = [];
        chain = [];
    else
        if ~isempty(chain)
            if data{1, 5}(i) == chain.id
                chain.C = [chain.C data{1, 2}(i)];
                chain.priority = [chain.priority data{1, 4}(i)];
            else
                chainset = [chainset; chain];
                chain = struct('id', data{1, 5}(i), 'T', data{1, 1}(i), 'C', data{1, 2}(i), 'D', data{1, 3}(i), 'priority', data{1, 4}(i));
            end
        else
            chain = struct('id', data{1, 5}(i), 'T', data{1, 1}(i), 'C', data{1, 2}(i), 'D', data{1, 3}(i), 'priority', data{1, 4}(i));
        end

    end
end

%statistics
schedulable = 0;
for i = 1 : size(resultset, 1)
    schedulable = schedulable + resultset(i).SCHED;
end
ratio = schedulable/size(resultset, 1);