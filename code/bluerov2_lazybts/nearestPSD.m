function P = nearestPSD(P, minimumEigenvalue)
%NEARESTPSD 对称化并截断微小负特征值，抑制浮点误差累积。
% 这里不是改变物理模型，而是修复 Kalman/Riccati 运算中约 1e-14 量级
% 的非对称和负特征值，使后续 Cholesky 与半正定支配判断稳定。

if nargin < 2, minimumEigenvalue = 0; end
P = (P + P')/2;
% 大多数 Riccati/Kalman 矩阵已经满足目标下界。先用 Cholesky 做 O(n^3)
% 但常数较小的快速判定；只有失败时才执行会重构矩阵的完整特征分解。
n=size(P,1);
[~,flag]=chol(P-minimumEigenvalue*eye(n));
if flag==0
    return
end
[V,D] = eig(P);
d = real(diag(D));
d(d < minimumEigenvalue) = minimumEigenvalue;
P = real(V * diag(d) * V');
P = (P + P')/2;
end
