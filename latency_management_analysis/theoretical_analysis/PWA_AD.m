function [R, S, SCHED] = PWA_AD(chainset, M, PRIO, CG_enabled)
SCHED = false;
%MAX_EFFORT = 10;

% initialize slack time for callbacks
deadlines = [];
for i = 1 : size(chainset, 1)
    deadlines = [deadlines; chainset(i).D];
end
S = zeros(size(chainset, 1), 1);
S_prev = S;
while true  % loop for updating slack time s

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

            % (second term) : interference from all interferring chains and
            % itself
            for i = 1 : size(chainset, 1)
                T = chainset(i).T;
                D = chainset(i).D;
                C = sum(chainset(i).C);     %C of the chain? Does it have the same issue of m sequetial callbacks.
                if PRIO
                    if chainset(i).priority(1) <= chainset(k).priority(1)
                        intf = double(intf) + Interference(l, D-C, T, C);
                    end
                else
                    intf = double(intf) + Interference(l, D-C, T, C);
                end
            end


            % add them all together
            if ~PRIO
                W = M * double(E_k) + double(intf) - sum(chainset(k).C);
            else
                exclusive_chainset = [];
                for i = 1: size(chainset, 1)
                    if i ~= k
                        exclusive_chainset = [exclusive_chainset; chainset(i)];
                    end
                end
                B = MLP(M, exclusive_chainset, chainset(k).priority(1), l);
                W = M * double(E_k) + double(intf) - sum(chainset(k).C) + double(B);

            end

            %%%%%%%%%%%%%%%%% Callback group test   %%%%%%%%%%%%%%%%
            if CG_enabled
                retval = 0;
                for j = 1:size(chainset(k).C, 2)
                    %                 retval = retval + strangers_in_group(chainset, k, j, PRIO, l);
                    retval = retval + fixed_sin(chainset, k, j, PRIO, l, 0);

                end

                W = W + M * retval;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            


            % break condition that a task is schedulable or not
            if W < 0
                l = l + 1;      %Is it OK to increase by 1? because W is negative
            elseif W < M*l
                % schedulable and the response time is l+C_k-1
                R(k) = l + chainset(k).C(end) - 1;
                L(k) = l;
                break;
                %elseif l > chainset(k).D-sum(chainset(k).C)+1      % only need
                %to consider last callback
            elseif l > chainset(k).D-chainset(k).C(end)+1

                R(k) = inf;
                L(k) = l;
                break;
            else
                % update l value
                l = double(1 + floor(1/M*(W)));

            end


        end
    end

    % check taskset is schedulable or not
    if ismember(inf, R)     % non-schedulable set
        break;
    else
        S = deadlines - R';
        if S == S_prev
            SCHED = true;
            break;
        end
    end
    S_prev = S;
end
end

% calculate W
function w = Interference(l, alpha, T, C)
if (l-floor(double((l+alpha))/T)*T) < 0
    w = floor(double((l+alpha))/T)*C + C;
    %w = -1;
else
    w = floor(double((l+alpha))/T)*C + min(C, l-floor(double((l+alpha))/T)*T);
end
end

function retval = MLP(M, chainset, self_priority, l)
possible_lp = [];
retval = 0;
for k = 1 : size(chainset, 1)    %Loop over chains
    for j = 1:size(chainset(k).C, 2)
        if chainset(k).priority(j) > self_priority
            b = min(chainset(k).C(j)-1, l);
            possible_lp = [possible_lp; double(b)];
        end
    end
    %can limit the instances by choosing the largest from each chain
end

n = min(M,size(possible_lp, 1));

mlp = maxk(possible_lp, n);

for c = 1 : size(mlp,1)
    retval = retval + mlp(c);
end
end
