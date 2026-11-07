function validateLBTSSetup(graph, options)
%VALIDATELBTSSETUP 在昂贵搜索前尽早报告接口或参数错误。

n=size(graph.Vertices,1);
assert(n>=1,'LBTS:EmptyGraph','trajectoryGraph 不含顶点。');
assert(isfield(graph.param,'model') && isfield(graph.param,'world') && isfield(graph.param,'metric') && isfield(graph.param.metric,'weights'),'LBTS:InvalidGraphParameters','trajectoryGraph.param 必须包含 model、world 和 metric.weights。');
assert(all(size(options.initialCovariance)==[12 12]),'initialCovariance 必须为 12x12。');
assert(all(size(options.initialEstimationCovariance)==[12 12]),'initialEstimationCovariance 必须为 12x12。');
assert(all(size(options.processNoiseQc)==[12 12]),'processNoiseQc 必须为 12x12。');
assert(all(size(options.lqr.Q)==[12 12]) && all(size(options.lqr.Qf)==[12 12]) && all(size(options.lqr.R)==[8 8]),'LQR 的 Q/Qf/R 维度错误。');
assert(options.chanceConstraint>=0 && options.chanceConstraint<=1,'chanceConstraint 必须位于 [0,1]。');
assert(options.chance.numSamples>=1 && options.chance.numSamples==round(options.chance.numSamples),'chance.numSamples 必须为正整数。');
indices=options.sensor.measurementIndices(:)';
assert(~isempty(indices) && all(indices>=1) && all(indices<=12) && all(indices==round(indices)) && numel(unique(indices))==numel(indices),'sensor.measurementIndices 必须是不重复的 1..12 整数。');
assert(all(size(options.cost.covarianceMatrix)==[12 12]),'cost.covarianceMatrix 必须为 12x12。');
assert(options.cost.covarianceWeight>=0,'covarianceWeight 必须非负，否则 Dijkstra 启发式不再保证是下界。');
assert(options.search.maxExpansions>=1 && options.search.maxPoppedEdges>=1 && options.search.maxBeliefsPerVertex>=1 && options.search.maxWallTime>0,'搜索容量/时间参数必须为正数。');

P=(options.initialCovariance+options.initialCovariance')/2;
Pt=(options.initialEstimationCovariance+options.initialEstimationCovariance')/2;
tol=options.numerics.psdTolerance;
if min(eig(P)) < -tol || min(eig(Pt)) < -tol || min(eig(P-Pt)) < -tol
    error('LBTS:InvalidInitialBelief','要求 P0、Ptilde0、P0-Ptilde0 均为半正定矩阵。');
end

% 当前图接口的每条边均必须含名义状态、推进器控制和时间。
for i = 1:n
    for edgeNumber=1:numel(graph.Edges{i})
        j=graph.Edges{i}(edgeNumber);
        cost=graph.EdgesCost{i}(edgeNumber);
        if ~isfinite(cost) || cost<0
            error('LBTS:InvalidEdgeCost','边 %d->%d 的代价必须有限且非负。',i,j);
        end
        edge = graph.EdgeData{i}{edgeNumber};
        if ~iscell(edge) || numel(edge)<4 || isempty(edge{1}) || isempty(edge{3}) || ~isstruct(edge{4})
            error('LBTS:MissingEdgeControl','边 %d->%d 必须为 EdgeData{i}{k}={xbar,N,ubar,info}。',i,j);
        end
        xbar=edge{1}; ubar=edge{3};
        if size(xbar,1)~=12 || size(ubar,1)~=8 || size(xbar,2)~=size(ubar,2)+1
            error('LBTS:InvalidEdgeData','边 %d->%d 的 xbar/ubar 维度应为 12x(N+1) 和 8xN。',i,j);
        end
        assert(isfield(edge{4},'time'), 'LBTS:MissingEdgeTime','边 %d->%d 的 info 必须提供 time。',i,j);
    end
end
end
