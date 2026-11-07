function propagation = propagateBelief(edge, P0, Ptilde0, ~, world, ...
        options, linearizer, gaussianSamples)
%PROPAGATEBELIEF 沿一条 BlueROV2 名义边传播分离式高斯信念。
%
% 设实际状态协方差为 P，估计误差协方差为 Ptilde，并定义
% Phat=P-Ptilde。沿名义轨迹线性化离散动力学：
%   delta x_{k+1}=A_k delta x_k+B_k delta u_k+w_k,
%   delta u_k=-K_k delta xhat_k.
% 分离式协方差传播为
%   P^-_{k+1}=A Ptilde A' + (A-BK) Phat (A-BK)' + Qk,
%   Ptilde^- = A Ptilde A' + Qk,
% 随后使用位置相关观测模型执行 Kalman 更新得到 Ptilde_{k+1}。
% P 表示闭环实际分布，Ptilde 表示给定观测后的估计误差；二者都参与
% belief-node 支配关系，避免只比较总协方差而丢失未来可观测性信息。

propagation = struct('success', false, 'message', '', 'P', [], ...
    'Ptilde', [], 'PSequence', {{}}, 'PtildeSequence', {{}}, ...
    'gains', {{}}, 'kalmanGains', {{}}, 'A', {{}}, 'B', {{}}, ...
    'measurementMatrices', {{}}, 'measurementCovariances', {{}}, ...
    'time', [], ...
    'risk', inf, 'riskByStep', [], 'covarianceCost', inf);

[xbar, ubar, time] = unpackEdge(edge);
N = size(xbar,2)-1;
if N < 1 || size(ubar,2) ~= N
    propagation.message = '边的状态节点数与控制区间数不一致。';
    return
end
dt = diff(time);
if any(~isfinite(dt)) || any(dt <= 0)
    propagation.message = '边时间戳必须严格递增。';
    return
end

% 先沿整条边求离散 Jacobian，再反向 Riccati 递推反馈增益。
A = cell(1,N); B = cell(1,N); Kfeedback = cell(1,N);
for k = 1:N
    [~, A{k}, B{k}] = evaluateRK4Linearization( ...
        linearizer, xbar(:,k), ubar(:,k), dt(k));
    if any(~isfinite(A{k}(:))) || any(~isfinite(B{k}(:)))
        propagation.message = sprintf('第 %d 步动力学 Jacobian 含非有限值。', k);
        return
    end
end
S = options.lqr.Qf;
for k = N:-1:1
    H = options.lqr.R + B{k}'*S*B{k};
    G = B{k}'*S*A{k};
    % pinv 只在病态 H 上使用；正常正定 R 情形采用反斜杠更准确。
    if rcond(H) < 1e-12
        Kfeedback{k} = pinv(H)*G;
    else
        Kfeedback{k} = H\G;
    end
    F = A{k}-B{k}*Kfeedback{k};
    S = options.lqr.Q + Kfeedback{k}'*options.lqr.R*Kfeedback{k} + F'*S*F;
    S = (S+S')/2;
end

P = nearestPSD(P0, options.numerics.minEigenvalue);
Ptilde = nearestPSD(Ptilde0, options.numerics.minEigenvalue);
Psequence = cell(1,N+1); PtildeSequence = cell(1,N+1);
KalmanGains=cell(1,N);
CSequence=cell(1,N);
RSequence=cell(1,N);
Psequence{1}=P; PtildeSequence{1}=Ptilde;
covarianceIntegral = 0;
for k = 1:N
    Phat = nearestPSD(P-Ptilde, 0);
    Qk = options.processNoiseQc * dt(k);
    F = A{k}-B{k}*Kfeedback{k};
    Pnext = A{k}*Ptilde*A{k}' + F*Phat*F' + Qk;
    Pminus = A{k}*Ptilde*A{k}' + Qk;

    [C,R] = measurementModelAtState(xbar(:,k+1), options.sensor);
    innovation = C*Pminus*C' + R;
    L = (Pminus*C')/innovation;
    % 这些量不仅用于规划协方差，也定义了 Monte Carlo/实机执行时应采用
    % 的同一套时变滤波器。只计算一次并随最终路径导出。
    KalmanGains{k}=L;
    CSequence{k}=C;
    RSequence{k}=R;
    IminusLC = eye(12)-L*C;
    % Joseph 形式比 (I-LC)Pminus 在有限精度下更能保持半正定性。
    PtildeNext = IminusLC*Pminus*IminusLC' + L*R*L';
    Pnext = nearestPSD(Pnext, options.numerics.minEigenvalue);
    PtildeNext = nearestPSD(PtildeNext, options.numerics.minEigenvalue);

    % 理论上 P-Ptilde 半正定；若仅有舍入级负值，则投影修复。若偏差
    % 显著，仍返回失败，防止错误模型产生虚假的低不确定度路径。
    difference = (Pnext-PtildeNext + (Pnext-PtildeNext)')/2;
    [~,differenceFlag]=chol(difference + ...
        100*options.numerics.psdTolerance*eye(12));
    if differenceFlag~=0 && ...
            min(real(eig(difference,'vector'))) < ...
            -100*options.numerics.psdTolerance
        propagation.message = sprintf('第 %d 步出现 P-Ptilde 非半正定。', k);
        return
    end
    PhatNext = nearestPSD(difference, 0);
    Pnext = nearestPSD(PtildeNext+PhatNext, options.numerics.minEigenvalue);

    covarianceIntegral = covarianceIntegral + dt(k) * ...
        trace(options.cost.covarianceMatrix*Pnext);
    P=Pnext; Ptilde=PtildeNext;
    Psequence{k+1}=P; PtildeSequence{k+1}=Ptilde;
end

chanceSettings=options.chance;
chanceSettings.stopThreshold=options.chanceConstraint;
[risk, riskByStep] = chanceCollisionProbability(xbar, Psequence, world, ...
    gaussianSamples, options.chance.includeNominalPoint, chanceSettings);
propagation.success = all(isfinite(P(:))) && all(isfinite(Ptilde(:)));
propagation.P = P;
propagation.Ptilde = Ptilde;
propagation.PSequence = Psequence;
propagation.PtildeSequence = PtildeSequence;
propagation.gains = Kfeedback;
propagation.kalmanGains = KalmanGains;
propagation.A = A;
propagation.B = B;
propagation.measurementMatrices = CSequence;
propagation.measurementCovariances = RSequence;
propagation.time = time;
propagation.risk = risk;
propagation.riskByStep = riskByStep;
propagation.covarianceCost = options.cost.covarianceWeight*covarianceIntegral;
end

function [xbar, ubar, time] = unpackEdge(edge)
xbar = edge{1}; ubar = edge{3};
time=reshape(edge{4}.time,1,[]);
end
