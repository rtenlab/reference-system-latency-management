function [retval] = fixed_sin(chainset, in_chain_id, in_callback_id, PRIO, delta, CD)


retval = 0;

% group1 = [1 1;1 2;1 3; 2 1;2 2];
% group2 = [2 3;2 4;2 5; 3 1; 3 2];

group1 = [1 1; 1 2; 2 1; 2 2; 2 3; 2 4];


% group1 = [5 1; 5 2; 5 3; 5 4];
% group2 = [4 1; 4 2; 4 3];
% group3 = [3 1; 3 2; 3 3; 3 4];
% group4 = [2 1;2 2; 2 3;2 4];
% group5 = [1 1; 1 2];

% group1 = [1 2; 2 2; 2 3; 2 4];
% group1 = [4 3; 5 3; 5 4];
% group2 = [4 1; 4 2; 4 3; 1 1; 1 2];
% group3 = [3 1; 3 2; 3 3; 3 4; 2 3; 2 4];

in_id = [in_chain_id in_callback_id];
group = [];

if ismember(in_id, group1, 'rows')
    group = group1;
% elseif ismember(in_id, group2, 'rows')
%     group = group2;
% elseif ismember(in_id, group3, 'rows')
%     group = group3;
% elseif ismember(in_id, group4, 'rows')
%     group = group4;
% elseif ismember(in_id, group5, 'rows')
%     group = group5;
end

if ~isempty(group)
    for i=1:size(group, 1)
        groupmate = group(i,:);
        g_chain_id = groupmate(1);
        g_callback_id = groupmate(2);
        if ~CD || g_chain_id ~= in_chain_id
            D = chainset(g_chain_id).D;
            T = chainset(g_chain_id).T;
            C = sum(chainset(g_chain_id).C);
            instance_n = ceil(double((delta + D - C)/T));
            if ~PRIO
                retval = retval + instance_n * chainset(g_chain_id).C(g_callback_id);
            else
                if chainset(g_chain_id).priority(g_callback_id) > chainset(in_chain_id).priority(in_callback_id)
                    retval = retval + instance_n * chainset(g_chain_id).C(g_callback_id);
                end
            end

        end


    end

end


end
