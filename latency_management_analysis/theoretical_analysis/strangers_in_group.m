function [retval] 
retval = 0;

nc = size(chainset, 1);     % Number of chains
n = size(chainset(1).C, 2); % Number of callbacks per chain     *** Only works when num of callbacks per chain are the same
ng = 5;                     % Number of callback group
groups = [];
randoms = [];


% Assign callbacks to group randomly
% chain_id = ceil(id/n)
% callback_id = (id mod n) + 1 
for i=1:ng
    for j = 1: ceil((nc*n)/ng)
        id = randi([1,n*nc]);
        while ismember(id, randoms)
            id = randi([1,n*nc]);
        end
        randoms = [randoms; id];
        groups(i,j) = id;
    end
end

in_id = (in_chain_n -1) * n + in_callback_n;
[row, ~] = find(groups==in_id);
groupmates = groups(row,:);

for i = 1: size(groupmates, 2)
    groupmate = groupmates(i);

    if groupmate ~= in_id
        g_chain_n = ceil(groupmate/n);
        g_callback_n = mod(groupmate, n) + 1;

        if g_chain_n ~= in_chain_n
            if ~PRIO
                retval = retval + chainset(g_chain_n).C(g_callback_n);
            else
                if chainset(g_chain_n).priority(g_callback_n) > chainset(in_chain_n).priority(in_callback_n)
                    D = chainset(in_chain_n).D;
                    T = chainset(in_chain_n).T;
                    C = sum(chainset(g_chain_n).C);
                    instance_n = ceil(double((delta + D - C)/T));
                    retval = retval + instance_n * chainset(g_chain_n).C(g_callback_n);
                end
            end
        end
    end
end

end
