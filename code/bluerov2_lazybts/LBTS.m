function result = LBTS(trajectoryGraphFile, startState, goalState, userOptions)
%LBTS BlueROV2 trajectoryGraph 上的 Lazy Belief Tree Search 主程序。
%
% 用法：
%   result = LBTS('trajectoryGraph.mat', xStart, xGoal);
%   result = LBTS(..., options);
%
% 输入：
%   trajectoryGraphFile  已生成的 BlueROV2 kinodynamic PRM MAT 文件；
%   startState/goalState 12x1 状态，顺序与 bluerov2Dynamics 一致；
%   userOptions          可选，仅覆盖 defaultLBTSOptions 中给定的字段。
% 输出 result 的关键字段：
%   success、cost、vertexPath、nominalTrajectory、controlTrajectory、
%   covarianceSequence、edgeRisks、statistics。
%
% 算法分成两个层次：trajectoryGraph 保存确定性最优控制边；belief tree
% 允许同一图顶点拥有多个 (P,Ptilde,cost) 节点。候选边先按
%   lowerBound = costToCome + nominalEdgeCost + DijkstraCostToGoal
% 排序，只有弹出时才做名义碰撞检查、非线性线性化、Kalman 协方差传播
% 和机会约束检查，故称为 Lazy BTS。与参考脚本“首个终点即停止”相比，
% 默认保留 incumbent，并搜索到队列下界不可能改善 incumbent 为止。

if nargin<1 || isempty(trajectoryGraphFile), trajectoryGraphFile='trajectoryGraph.mat'; end
if nargin<4, userOptions=struct(); end
options=mergeStructRecursive(defaultLBTSOptions(),userOptions);
totalClock=tic;

% KPRM 目录是只读依赖：提供动力学、参数和起终点 meanControl。-begin
% 防止 MATLAB 路径上旧 double-integrator 的同名 meanControl 被误调用。
kprmPath=char(options.kprmPath);
if ~isfile(fullfile(kprmPath,'bluerov2Dynamics.m')) || ...
        ~isfile(fullfile(kprmPath,'meanControl.m'))
    error('LBTS:KPRMPath', 'kprmPath 不是有效的 bluerov2_kprm 目录：%s',kprmPath);
end
addpath(kprmPath,'-begin');

graph=loadTrajectoryGraph(trajectoryGraphFile);
if isfield(options,'world') && ~isempty(options.world), graph.param.world=options.world; end
validateLBTSSetup(graph,options);
attachmentReport=struct();
if options.attachQuery
    if nargin<3 || isempty(startState) || isempty(goalState)
        error('LBTS:MissingQuery', 'attachQuery=true 时必须提供起点和终点状态。');
    end
    [graph,startIndex,goalIndex,attachmentReport]=attachQueryStates( ...
        graph,startState,goalState,options);
else
    % 不连接查询状态时，第二、三个参数解释为已有图顶点索引。
    startIndex=startState; goalIndex=goalState;
    if ~isscalar(startIndex)||~isscalar(goalIndex)
        error('LBTS:QueryIndex','attachQuery=false 时 startState/goalState 必须是顶点索引。');
    end
    nGraph=size(graph.Vertices,1);
    if any([startIndex goalIndex]<1) || any([startIndex goalIndex]>nGraph) || ...
            any([startIndex goalIndex]~=round([startIndex goalIndex]))
        error('LBTS:QueryIndex','起终点索引必须是 1 到 %d 的整数。',nGraph);
    end
end
heuristic=backwardDijkstra(graph,goalIndex);
if ~isfinite(heuristic(startIndex))
    statistics=emptyStatistics(); statistics.termination='nominal_graph_disconnected';
    statistics.totalElapsedSeconds=toc(totalClock);
    result=assembleLBTSResult(false,[0 0],cell(size(graph.Vertices,1),1), ...
        graph,heuristic,statistics,options,startIndex,goalIndex,attachmentReport);
    return
end

linearizer=makeRK4Linearizer(graph.param.model,options.linearization);
Z=standardGaussianSamples(3,options.chance.numSamples,options.chance.rngSeed);
n=size(graph.Vertices,1);
beliefs=cell(n,1);
initial=makeBelief(options.initialCovariance, ...
    options.initialEstimationCovariance,0,heuristic(startIndex),[0 0],[]);
beliefs{startIndex}=initial;

queue=emptyQueue();
queue=appendOutgoing(queue,graph,startIndex,1,beliefs,heuristic);
% 碰撞有效性与 EdgeData 同序，只为真实边分配一个 int8 标志。
meanValidity=cellfun(@(e) zeros(size(e),'int8'),graph.Edges, ...
    'UniformOutput',false); % 0=未知，1=有效，-1=碰撞
incumbentCost=inf; incumbentId=[0 0];
statistics=emptyStatistics();
searchClock=tic;

while ~isempty(queue) && statistics.expandedEdges<options.search.maxExpansions && statistics.poppedEdges<options.search.maxPoppedEdges && toc(searchClock)<options.search.maxWallTime
    [~,qIndex]=min([queue.priority]);
    candidate=queue(qIndex); queue(qIndex)=[];
    statistics.poppedEdges=statistics.poppedEdges+1;

    if candidate.priority>=incumbentCost-options.search.boundTolerance
        statistics.termination='incumbent_bound_proved';
        break
    end
    sourceNode=beliefs{candidate.sourceVertex}(candidate.sourceBelief);
    if ~sourceNode.active, statistics.staleEdges=statistics.staleEdges+1; continue; end
    i=candidate.sourceVertex; j=candidate.targetVertex;
    edge=graph.EdgeData{i}{candidate.edgeIndex};

    % 第一层惰性检查只看名义轨迹，并缓存图边结果。它与 belief 无关，
    % 因而同一 trajectoryGraph 边被不同 belief 尝试时无需重复检测。
    if meanValidity{i}(candidate.edgeIndex)==0
        statistics.meanCollisionChecks=statistics.meanCollisionChecks+1;
        if isMeanTrajectoryCollisionFreeLBTS(edge{1},graph.param.world)
            meanValidity{i}(candidate.edgeIndex)=1;
        else
            meanValidity{i}(candidate.edgeIndex)=-1;
        end
    end
    if meanValidity{i}(candidate.edgeIndex)<0
        statistics.meanCollisionRejected=statistics.meanCollisionRejected+1;
        continue
    end

    statistics.expandedEdges=statistics.expandedEdges+1;
    propagation=propagateBelief(edge,sourceNode.P,sourceNode.Ptilde, ...
        graph.param.model,graph.param.world,options,linearizer,Z);
    if ~propagation.success
        statistics.numericalRejected=statistics.numericalRejected+1;
        continue
    end
    if propagation.risk>options.chanceConstraint
        statistics.chanceRejected=statistics.chanceRejected+1;
        continue
    end
    newCost=sourceNode.costToCome+candidate.nominalCost+propagation.covarianceCost;
    if isBeliefDominated(beliefs{j},newCost,propagation.P, ...
            propagation.Ptilde,options.dominance)
        statistics.dominanceRejected=statistics.dominanceRejected+1;
        continue
    end
    if numel(beliefs{j})>=options.search.maxBeliefsPerVertex
        statistics.vertexCapacityRejected=statistics.vertexCapacityRejected+1;
        continue
    end

    newIndex=numel(beliefs{j})+1;
    priority=newCost+heuristic(j);
    newNode=makeBelief(propagation.P,propagation.Ptilde, ...
        newCost,priority,[i,candidate.sourceBelief],propagation);
    if isempty(beliefs{j})
        beliefs{j}=newNode;
    else
        beliefs{j}(newIndex)=newNode;
    end
    statistics.acceptedBeliefs=statistics.acceptedBeliefs+1;

    if j==goalIndex
        if newCost<incumbentCost
            incumbentCost=newCost; incumbentId=[j,newIndex];
            statistics.incumbentUpdates=statistics.incumbentUpdates+1;
            if options.search.verbose
                fprintf('LBTS incumbent: cost %.6g after %d propagated edges.\n', ...
                    incumbentCost,statistics.expandedEdges);
            end
        end
        if options.search.stopAtFirstSolution
            statistics.termination='first_solution'; break
        end
    else
        queue=appendOutgoing(queue,graph,j,newIndex,beliefs,heuristic);
    end
end

if isempty(statistics.termination)
    if statistics.expandedEdges>=options.search.maxExpansions
        statistics.termination='max_expansions';
    elseif statistics.poppedEdges>=options.search.maxPoppedEdges
        statistics.termination='max_popped_edges';
    elseif toc(searchClock)>=options.search.maxWallTime
        statistics.termination='max_wall_time';
    elseif isempty(queue)
        statistics.termination='queue_empty';
    else
        statistics.termination='stopped';
    end
end
statistics.elapsedSeconds=toc(searchClock);
statistics.totalElapsedSeconds=toc(totalClock);
statistics.remainingQueueSize=numel(queue);
statistics.linearizationMode=linearizer.mode;
success=isfinite(incumbentCost);
result=assembleLBTSResult(success,incumbentId,beliefs,graph,heuristic, ...
    statistics,options,startIndex,goalIndex,attachmentReport);
if options.search.verbose
    fprintf(['LBTS finished: success=%d, expanded=%d, reason=%s, ' ...
        'search=%.3fs, total=%.3fs.\n'],success,statistics.expandedEdges, ...
        statistics.termination,statistics.elapsedSeconds, ...
        statistics.totalElapsedSeconds);
end
end

function node=makeBelief(P,Ptilde,cost,priority,parent,incoming)
node=struct('P',P,'Ptilde',Ptilde,'costToCome',cost,'priority',priority, ...
    'parent',parent,'incoming',incoming,'active',true);
end

function queue=emptyQueue()
queue=struct('sourceVertex',{},'sourceBelief',{},'targetVertex',{}, ...
    'edgeIndex',{},'priority',{},'nominalCost',{});
end

function queue=appendOutgoing(queue,graph,vertex,beliefIndex,beliefs,heuristic)
source=beliefs{vertex}(beliefIndex);
for k=1:numel(graph.Edges{vertex})
    target=graph.Edges{vertex}(k); cost=graph.EdgesCost{vertex}(k);
    if ~isfinite(cost)||cost<0||~isfinite(heuristic(target)), continue; end
    entry=struct('sourceVertex',vertex,'sourceBelief',beliefIndex, ...
        'targetVertex',target,'edgeIndex',k, ...
        'priority',source.costToCome+cost+heuristic(target), ...
        'nominalCost',cost);
    queue(end+1)=entry; %#ok<AGROW>
end
end

function s=emptyStatistics()
s=struct('poppedEdges',0,'expandedEdges',0,'staleEdges',0, ...
    'meanCollisionChecks',0,'meanCollisionRejected',0, ...
    'chanceRejected',0,'dominanceRejected',0,'numericalRejected',0, ...
    'vertexCapacityRejected',0,'acceptedBeliefs',0,'incumbentUpdates',0, ...
    'remainingQueueSize',0,'elapsedSeconds',0,'totalElapsedSeconds',0, ...
    'linearizationMode','', ...
    'termination','');
end
