function results = find_WOT_of_callbacks(names, exectime)

results = [];
callbacks = [];
for i = 1: size(names, 1)
%     if i>1560
%         disp("error");
%     end
    if isempty(callbacks)
        callbacks = [callbacks; names(i) exectime(i)];
    else
        if any(contains(callbacks(:,1), names(i)))
            index = find(contains(callbacks(:,1), names(i)));
            if size(index, 1) > 1
                disp('index is wrong');
            end
            callbacks{index,2} = [callbacks{index,2}; exectime(i)];

        else
            callbacks = [callbacks; names(i) exectime(i)];
        end
    end
end

for i = 1: size(callbacks(:,1))
    callback = cell2mat(callbacks(i, 2:end));
    callback = sort(callback, "descend");
    outliers = isoutlier(callback);
    c = 1;
    while outliers(c)~=0
        c = c + 1;
    end
    WOT = callback(c);
    results = [results; callbacks(i,1) WOT];
end



end