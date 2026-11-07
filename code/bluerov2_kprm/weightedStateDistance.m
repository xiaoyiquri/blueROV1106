function distance = weightedStateDistance(x, Y, weights)
%WEIGHTEDSTATEDISTANCE Wrapped weighted distance from x to rows of Y.
%
% x 是 1x12 查询状态，Y 的每一行是一个候选状态。该距离只用于近邻
% 排序，不是边代价；真正的边代价由 meanControl 的非线性最优控制
% 目标给出。weights 用于协调米、弧度、m/s 和 rad/s 的量纲差异。

x = x(:)';
assert(size(Y, 2) == 12 && numel(x) == 12);
delta = Y - x;
% 欧拉角的物理状态以 2*pi 为周期，使用 atan2(sin,cos) 得到最短角差。
delta(:, 4:6) = atan2(sin(delta(:, 4:6)), cos(delta(:, 4:6)));
w = weights(:)';
distance = sqrt(sum((delta .* w).^2, 2));
end
