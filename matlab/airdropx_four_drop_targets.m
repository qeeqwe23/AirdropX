function [targetN, targetE] = airdropx_four_drop_targets(centerN, centerE, dropIndex, offsetN, offsetE)
%AIRDROPX_FOUR_DROP_TARGETS Return the target point for one or more drops.

if nargin < 4 || isempty(offsetN)
    offsetN = zeros(4, 1);
end
if nargin < 5 || isempty(offsetE)
    offsetE = zeros(4, 1);
end

offsetN = double(offsetN(:));
offsetE = double(offsetE(:));
dropIndex = max(1, round(double(dropIndex(:))));
n = numel(dropIndex);
targetN = double(centerN) * ones(n, 1);
targetE = double(centerE) * ones(n, 1);

for i = 1:n
    idx = dropIndex(i);
    if idx <= numel(offsetN)
        targetN(i) = targetN(i) + offsetN(idx);
    end
    if idx <= numel(offsetE)
        targetE(i) = targetE(i) + offsetE(idx);
    end
end
end
