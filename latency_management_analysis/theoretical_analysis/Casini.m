function [R, S, SCHED] = Casini(chainset)
SCHED = true;
S = [];
RT = [];
R = [];

% to update slack time, we need to calculate response time of chains
for k = 1 : chainset(end).id
    %timer callback
    R(1) = RT_callback(chainset, k, 1, 1, 1);     %chain_id = k, callback_id = 1, M =1, istimer = 1

    %regular callbacks
    for j=2:size(chainset(k).C, 2)
        R(j) = RT_callback(chainset, k, 1, 1, 0);
    end
    if ismember(inf, R)     % non-schedulable chain
        RT(k) = inf;
        continue;
    else
        RT(k) = sum(R);     
    end

end
if ismember(inf, RT)
    SCHED = false;
end

end


function W = RBF_timer(chainset, chain_id, callback_id, l)
W = ceil(l/chainset(chain_id).T) * chainset(chain_id).C(callback_id);       %rbf of the callback itself
for k = 1 : chainset(end).id
    for j = 1:size(chainset(k).C, 2)
        if chainset(k).priority(j) < chainset(chain_id).priority(callback_id)   %higher-priority chains
            T = chainset(k).T;
            C = chainset(k).C(j);
            W = W + double(double(ceil(double(l/T))) * C);
        elseif chainset(k).priority(j) > chainset(chain_id).priority(callback_id)   %lower-priority chains
            W = W + double (chainset(k).C(j));
        end
    end
end
end

function W = RBF_regular(chainset, chain_id, callback_id, l)
W = ceil(l/chainset(chain_id).T) * chainset(chain_id).C(callback_id);       %rbf of the callback itself
for k = 1 : chainset(end).id
    for j = 1:size(chainset(k).C, 2)
        if k~=chain_id && j~=callback_id
            T = chainset(k).T;
            C = chainset(k).C(j);
            W = W + double(double(ceil(double(l/T))) * C);
        end
    end
end
end

function R = RT_callback(chainset, chain_id, callback_id, M, istimer)
l = 1;
while true
    if istimer
        W = RBF_timer(chainset, chain_id, callback_id, l);
    else
        W = RBF_regular(chainset, chain_id, callback_id, l);
    end

    % break condition that a callback is schedulable or not
    if W < M*l
        % schedulable and the response time is l+C_k-1
        R = l + chainset(chain_id).C(callback_id) - 1;
        break;
    elseif l > chainset(chain_id).D-chainset(chain_id).C(callback_id)+1
        R = inf;
        break;
    else
        % update l value
        l = double(1 + floor(1/M*(W)));
    end
end

end

