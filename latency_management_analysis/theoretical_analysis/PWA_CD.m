function [R, S, SCHED] = PWA_CD(chainset, M, PRIO,CG_enabled, threadclass_budget)
SCHED = false;
%MAX_EFFORT = 10;
period_us = 1000;
% initialize slack time for callbacks
deadlines = [];
for i = 1 : size(chainset, 1)
    deadlines = [deadlines; chainset(i).D];
end
S = zeros(size(chainset, 1), 1);
S_prev = S;

%M = M * (threadclass_budget/period_us); % M threads, with identical budget
% while true  % loop for updating slack time s

    % to update slack time, we need to calculate response time of chains
    for k = 1 : chainset(end).id

        % initialize l value
        l = 1;
        %l = sum(chainset(k).C)-chainset(k).C(end);

        %effort =0;

        % (first term) : Resereved resource for the chainset(k)
        E_k = sum(chainset(k).C)-chainset(k).C(end) +1;

        while true
            W = 0.0; intf = 0.0;

            % (second term) : interference from all interferring chains
            for i = 1 : size(chainset, 1)
                if i ~= k       %Exclusive
                    T = chainset(i).T;
                    D = chainset(i).D;
                    C = sum(chainset(i).C);     %C of the chain? Does it have the same issue of m sequetial callbacks.
                    if PRIO
                        if chainset(i).priority(1) > chainset(k).priority(1)
                            intf = double(intf) + Interference(l, D-C, T, C);
                        end
                    else
                        intf = double(intf) + Interference(l, D-C, T, C);
                    end
                end
            end


            % add them all together
            if ~PRIO
                W = M * double(E_k) + double(intf);
            else
                exclusive_chainset = [];
                for i = 1: size(chainset, 1)
                    if i ~= k
                        exclusive_chainset = [exclusive_chainset; chainset(i)];
                    end
                end
                B = MLP(M, exclusive_chainset, chainset(k).priority(1), l);
                W = M * double(E_k) + double(intf) + double(B);

            end

            % break condition that a task is schedulable or not
            if W < 0
                l = l + 1;      %Is it OK to increase by 1? because W is negative            
            elseif W < M * sbf(l, period_us, threadclass_budget)
            %elseif W < M*l
                %disp(M)
                %disp(sbf(l, period_us, threadclass_budget))
                %disp(l)
                %elseif W < M*l
                % schedulable and the response time is l+C_k-1
                %R(k) = l + chainset(k).C(end) - 1;
                R(k) = l + pisbf_v2(l, chainset(k).C(end) -1, period_us, threadclass_budget);
                %R(k) = l;
                L(k) = l;
                break;
            elseif l > 20e6
                R(k) = inf;
                L(k) = l;
                break;                 
            else
                l = double(l + floor(W/M));
            end
        end
    end
    
%     % check taskset is schedulable or not
%     if ismember(inf, R)     % non-schedulable set
%         break;
%     else
%         S = deadlines - R';
%         if S == S_prev
%             SCHED = true;
%             break;
%         end
%     end
%     S_prev = S;
% end
end

% % calculate W
% function w = Interference(l, alpha, T, C)
% if (l-floor(double((l+alpha))/T)*T) < 0
%     w = floor(double((l+alpha))/T)*C + C;
%     %w = -1;
% else
%     w = floor(double((l+alpha))/T)*C + min(C, l-floor(double((l+alpha))/T)*T);
% end
% end

% Revised W with added alpha
function w = Interference(l, alpha, T, C)
    w = floor(double((l+alpha))/T)*C + min(C, l+alpha-floor(double((l+alpha))/T)*T);
end


function retval = MLP(M, chainset, self_priority, l)
possible_lp = [];
retval = 0;
for k = 1 : size(chainset, 1)    %Loop over chains
    each_chain_lp = [];
    for j = 1:size(chainset(k).C, 2)
        if chainset(k).priority(j) < self_priority
            b = min(chainset(k).C(j)-1, l);
            each_chain_lp = [each_chain_lp; double(b)];
        end
    end
    if isempty(each_chain_lp)
        continue;       %There is no lp callback in the chain(k)
    end
    possible_lp = [possible_lp; double(max(each_chain_lp))];     %Choose one callback of each chain

end

n = min(M,size(possible_lp, 1));

mlp = maxk(possible_lp, n);

for c = 1 : size(mlp,1)
    retval = retval + mlp(c);
end
end

function delta_delta_mid = pisbf(delta_delta, x, period_us, budget_us)
    delta_delta_min = 2 * (period_us - budget_us);
    delta_delta_max = delta_delta_min +2*x;
    delta_delta_mid = 0;
    while(delta_delta_max - delta_delta_min > 1e-6)
        delta_delta_mid = (delta_delta_min + delta_delta_max) /2;
        if sbf(delta_delta_mid, period_us, budget_us) < x
            delta_delta_min = delta_delta_mid;
        else
            delta_delta_max = delta_delta_mid;
        end
    end
end

function sbf_delta_delta = sbf(delta_delta, period_us, budget_us)

    if(delta_delta >= 2* (period_us - budget_us))
        sbf_delta_delta = (budget_us / period_us) * (delta_delta - 2 * (period_us - budget_us));
    else
        sbf_delta_delta = 0;
    end
end


function delta_delta_mid = pisbf_v2(delta_delta, x, period_us, budget_us)
    if(x > 0)
        delta_delta_mid = (period_us/budget_us)*x + 2*(period_us - budget_us);
    else
        delta_delta_mid = -1;
    end
end