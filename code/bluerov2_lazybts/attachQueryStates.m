function [graph,startIndex,goalIndex,report]=attachQueryStates(graph,startState,goalState,options)
%ATTACHQUERYSTATES 仅在内存中把查询起点和终点连接到 trajectoryGraph。
%
% 新增的有向边仍由 bluerov2_kprm/meanControl 求解，故满足同一套完整
% 6-DOF 动力学、推力/速度约束和代价定义。函数不会保存 graph，也不会
% 修改原 MAT 文件或 bluerov2_kprm 文件夹。

startState=startState(:); goalState=goalState(:);
assert(numel(startState)==12 && numel(goalState)==12,'起点和终点都必须是 12 维 BlueROV2 状态。');
nBase = size(graph.Vertices,1);
startIndex = nBase+1; goalIndex = nBase+2;

graph.Vertices(end+1,:) = startState';
graph.Vertices(end+1,:) = goalState';
graph.Edges{startIndex,1} = [];
graph.Edges{goalIndex,1} = [];
graph.Parents{startIndex,1} = [];
graph.Parents{goalIndex,1} = [];
graph.EdgesCost{startIndex,1} = [];
graph.EdgesCost{goalIndex,1} = [];
graph.EdgeData{startIndex,1}=cell(1,0);
graph.EdgeData{goalIndex,1}=cell(1,0);

startCandidates=nearestCandidates(startState,graph.Vertices(1:nBase,:),options.query,graph.param.metric.weights);
goalCandidates=nearestCandidates(goalState,graph.Vertices(1:nBase,:),options.query,graph.param.metric.weights);
report=struct('startAttempts',0,'startAccepted',0,'goalAttempts',0,'goalAccepted',0,'directAttempted',false,'directAccepted',false);

% 查询起点只需连向基础图；查询终点只需接收基础图入边。这保留了原图
% 的方向性，不假定水下机器人轨迹可以简单反向执行。
for j = startCandidates
    report.startAttempts = report.startAttempts+1;
    [ok,edge,cost] = solveQueryEdge(startState, graph.Vertices(j,:)', graph.param);
    if ok
        graph = insertDirectedEdge(graph,startIndex,j,edge,cost);
        report.startAccepted = report.startAccepted+1;
    end
end
for i = goalCandidates
    report.goalAttempts = report.goalAttempts+1;
    [ok,edge,cost] = solveQueryEdge(graph.Vertices(i,:)', goalState, graph.param);
    if ok
        graph = insertDirectedEdge(graph,i,goalIndex,edge,cost);
        report.goalAccepted = report.goalAccepted+1;
    end
end

if options.query.tryDirectConnection
    report.directAttempted = true;
    [ok,edge,cost] = solveQueryEdge(startState,goalState,graph.param);
    if ok
        graph = insertDirectedEdge(graph,startIndex,goalIndex,edge,cost);
        report.directAccepted = true;
    end
end
if options.query.verbose
    fprintf('LBTS query attachment: start %d/%d, goal %d/%d, direct=%d.\n',report.startAccepted,report.startAttempts,report.goalAccepted,report.goalAttempts,report.directAccepted);
end
end

function candidates = nearestCandidates(query, vertices, queryOptions, weights)
positionDistance = vecnorm(vertices(:,1:3)-query(1:3)',2,2);
candidates = find(positionDistance <= queryOptions.connectionRadius);
if isempty(candidates), candidates=[]; return; end
delta = vertices(candidates,:)-query(:)';
delta(:,4:6)=atan2(sin(delta(:,4:6)),cos(delta(:,4:6)));
distance = sqrt(sum((delta.*weights(:)').^2,2));
[~,order]=sort(distance,'ascend');
candidates = reshape(candidates(order(1:min(queryOptions.kNeighbors,numel(order)))),1,[]);
end

function [ok,edge,cost] = solveQueryEdge(x0,xf,param)
[xbar,cost,N,ubar,info]=meanControl(x0,xf,param);
ok=info.success && isfinite(cost) && isMeanTrajectoryCollisionFreeLBTS(xbar,param.world);
if ok
    edge={xbar,N,ubar,info};
else
    edge=[]; cost=inf;
end
end

function graph = insertDirectedEdge(graph,i,j,edge,cost)
% 防止直接连接与候选连接意外生成同一条重复边。
if any(graph.Edges{i}==j), return; end
graph.Edges{i}(end+1)=j;
graph.EdgesCost{i}(end+1)=cost;
graph.Parents{j}(end+1)=i;
graph.EdgeData{i}{end+1}=edge;
end
