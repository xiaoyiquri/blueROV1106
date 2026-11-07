function [C, R] = measurementModelAtState(x, sensor)
%MEASUREMENTMODELATSTATE 返回名义位置处的线性观测矩阵与噪声协方差。
%
% 当前采用 z=Cx+v。C 由 measurementIndices 选择状态分量；R 可随三维
% 区域改变，用于描述声学基站附近、视觉标志附近或浑浊区中的定位质量。

indices = sensor.measurementIndices(:)';
m = numel(indices);
C = zeros(m, 12);
C(sub2ind([m,12], 1:m, indices)) = 1;
R = covarianceFromSetting(sensor.defaultStd, [], indices, m);

position = x(1:3);
for k = 1:numel(sensor.regions)
    region = sensor.regions(k);
    if all(position >= region.min(:)) && all(position <= region.max(:))
        regionR = [];
        regionStd = [];
        if isfield(region, 'R'), regionR = region.R; end
        if isfield(region, 'std'), regionStd = region.std; end
        R = covarianceFromSetting(regionStd, regionR, indices, m);
    end
end
R = nearestPSD(R, 1e-14);
end

function R = covarianceFromSetting(stdValue, explicitR, indices, m)
if ~isempty(explicitR)
    R = explicitR;
    if any(size(R) ~= [m m])
        error('LBTS:SensorDimension', '区域 sensor.R 必须是 %d x %d。', m, m);
    end
    return
end
if isempty(stdValue)
    error('LBTS:SensorSetting', '传感器区域必须提供 std 或 R。');
end
stdValue = stdValue(:);
if numel(stdValue) == 12
    stdValue = stdValue(indices);
elseif numel(stdValue) ~= m
    error('LBTS:SensorDimension', 'sensor.std 长度必须为 12 或 %d。', m);
end
R = diag(stdValue.^2);
end
