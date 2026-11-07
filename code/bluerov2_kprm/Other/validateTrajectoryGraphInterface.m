function report = validateTrajectoryGraphInterface(trajectoryGraph)
%VALIDATETRAJECTORYGRAPHINTERFACE 验证 KPRM 到 LazyBTS 的数据契约。
%
% 本函数不修改图。它在保存 trajectoryGraph.mat 之前逐边确认：
%   1) Vertices 是 n x 12，状态顺序为
%      [N,E,D,roll,pitch,yaw,u,v,w,p,q,r]；
%   2) Edges{i}(k) 与 EdgesCost{i}(k) 一一对应，Parents 方向一致；
%   3) EdgeData{i}{k}={xbar,N,ubar,info}，与 Edges{i}(k) 同序；
%   4) xbar 为 12x(N+1)，ubar 为 8xN，且 info 提供有效时间；
%   5) 轨迹端点对应顶点，EdgesCost 对应 meanControl 的 info.cost。
%
% 这些检查覆盖 loadTrajectoryGraph 和 propagateBelief 真正使用的字段，
% 因而通过后可直接把输出 MAT 文件传给 BlueROV2 LazyBTS。

required = {'Vertices','Edges','Parents','EdgesCost','EdgeData','param'};
for k = 1:numel(required)
    assert(isfield(trajectoryGraph,required{k}), ...
        'trajectoryGraph 缺少字段 %s。', required{k});
end

V = trajectoryGraph.Vertices;
assert(isnumeric(V) && ismatrix(V) && size(V,2)==12 && ...
    all(isfinite(V(:))), 'Vertices 必须是有限数值组成的 n x 12 矩阵。');
n = size(V,1);
assert(iscell(trajectoryGraph.Edges) && numel(trajectoryGraph.Edges)==n, ...
    'Edges 必须含 n 个 cell。');
assert(iscell(trajectoryGraph.Parents) && numel(trajectoryGraph.Parents)==n, ...
    'Parents 必须含 n 个 cell。');
assert(iscell(trajectoryGraph.EdgesCost) && ...
    numel(trajectoryGraph.EdgesCost)==n, 'EdgesCost 必须含 n 个 cell。');
assert(iscell(trajectoryGraph.EdgeData) && ...
    numel(trajectoryGraph.EdgeData)==n, ...
    'EdgeData 必须含 n 个与邻接表对齐的 cell。');

edgeCount = 0;
for i = 1:n
    successors = reshape(trajectoryGraph.Edges{i},1,[]);
    costs = reshape(trajectoryGraph.EdgesCost{i},1,[]);
    assert(numel(successors)==numel(costs), ...
        '顶点 %d 的 Edges 与 EdgesCost 长度不相等。',i);
    assert(iscell(trajectoryGraph.EdgeData{i}) && ...
        numel(trajectoryGraph.EdgeData{i})==numel(successors), ...
        '顶点 %d 的 EdgeData 与 Edges 长度不相等。',i);
    assert(numel(unique(successors))==numel(successors), ...
        '顶点 %d 含重复后继。',i);

    for k = 1:numel(successors)
        j = successors(k);
        assert(j>=1 && j<=n && j==round(j), '边 %d 的终点索引非法。',i);
        assert(any(trajectoryGraph.Parents{j}==i), ...
            'Parents{%d} 中缺少边 %d->%d 的起点。',j,i,j);
        assert(isfinite(costs(k)) && costs(k)>=0, ...
            '边 %d->%d 的代价必须有限且非负。',i,j);

        edge = trajectoryGraph.EdgeData{i}{k};
        assert(iscell(edge) && numel(edge)>=4, ...
            '边 %d->%d 必须保存 {xbar,N,ubar,info}。',i,j);
        xbar = edge{1}; N = edge{2}; ubar = edge{3}; info = edge{4};
        assert(isnumeric(xbar) && size(xbar,1)==12, ...
            '边 %d->%d 的 xbar 必须为 12x(N+1)。',i,j);
        assert(isscalar(N) && N>=1 && N==round(N), ...
            '边 %d->%d 的 N 必须为正整数。',i,j);
        assert(isnumeric(ubar) && size(ubar,1)==8, ...
            '边 %d->%d 的 ubar 必须为 8xN。',i,j);
        assert(size(xbar,2)==N+1 && size(ubar,2)==N, ...
            '边 %d->%d 的 xbar、N、ubar 区间数量不一致。',i,j);
        assert(all(isfinite(xbar(:))) && all(isfinite(ubar(:))), ...
            '边 %d->%d 的轨迹或控制含 NaN/Inf。',i,j);
        assert(isstruct(info) && isfield(info,'cost') && isfinite(info.cost), ...
            '边 %d->%d 的 info.cost 缺失或无效。',i,j);
        assert(hasValidTime(info,N), ...
            '边 %d->%d 的 info.time/info.T 缺失或无效。',i,j);

        % meanControl 会把等价终点欧拉角放到最接近起点的 2*pi 分支，
        % 因此位置、速度直接比较，姿态误差先包络到 [-pi,pi]。
        startError = stateDifference(xbar(:,1),V(i,:)');
        goalError = stateDifference(xbar(:,end),V(j,:)');
        assert(norm(startError,inf)<=1e-6 && norm(goalError,inf)<=1e-6, ...
            '边 %d->%d 的轨迹端点与 Vertices 不一致。',i,j);
        costTolerance = 1e-8*max(1,abs(info.cost));
        assert(abs(costs(k)-info.cost)<=costTolerance, ...
            '边 %d->%d 的 EdgesCost 与 info.cost 不一致。',i,j);
        edgeCount = edgeCount+1;
    end
end

% 反向再检查一次，避免 Parents 中出现 Edges 不存在的幽灵边。
for j = 1:n
    parents = reshape(trajectoryGraph.Parents{j},1,[]);
    assert(numel(unique(parents))==numel(parents), 'Parents{%d} 含重复索引。',j);
    for i = parents
        assert(i>=1 && i<=n && any(trajectoryGraph.Edges{i}==j), ...
            'Parents 中的 %d->%d 在 Edges 中不存在。',i,j);
    end
end

if isfield(trajectoryGraph,'edge_count')
    assert(trajectoryGraph.edge_count==edgeCount, ...
        'edge_count=%d，但根据 Edges 统计得到 %d。', ...
        trajectoryGraph.edge_count,edgeCount);
end
report = struct('compatible',true,'vertexCount',n,'edgeCount',edgeCount, ...
    'checkedEdgeDataCount',edgeCount,'stateDimension',12,'controlDimension',8);
end

function valid = hasValidTime(info,N)
valid = false;
if isfield(info,'time') && isnumeric(info.time) && numel(info.time)==N+1
    t = reshape(info.time,1,[]);
    valid = all(isfinite(t)) && all(diff(t)>0);
elseif isfield(info,'T') && isscalar(info.T)
    valid = isfinite(info.T) && info.T>0;
end
end

function difference = stateDifference(a,b)
difference = a(:)-b(:);
difference(4:6) = atan2(sin(difference(4:6)),cos(difference(4:6)));
end
