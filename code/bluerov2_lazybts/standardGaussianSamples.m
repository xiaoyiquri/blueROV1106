function Z = standardGaussianSamples(dimension, count, seed)
%STANDARDGAUSSIANSAMPLES 生成固定、可复现的白化高斯样本。
% 同一组 Z 会用于所有候选边，构成 common random numbers，减少不同边
% 风险估计之间的蒙特卡洛抖动。正负成对样本也改善小样本下的对称性。

persistent cachedDimension cachedCount cachedSeed cachedZ
if ~isempty(cachedZ) && isequal(cachedDimension,dimension) && ...
        isequal(cachedCount,count) && isequal(cachedSeed,seed)
    Z=cachedZ;
    return
end
stream = RandStream('mt19937ar', 'Seed', seed);
halfCount = floor(count/2);
half = randn(stream, dimension, halfCount);
Z = [half, -half];
if size(Z,2) < count
    Z(:,end+1) = zeros(dimension,1);
end
Z = Z(:,1:count);
cachedDimension=dimension;
cachedCount=count;
cachedSeed=seed;
cachedZ=Z;
end
