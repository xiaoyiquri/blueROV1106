function result = assembleLBTSResult(success, goalId, beliefs, graph, ...
        heuristic, statistics, options, startIndex, goalIndex, attachmentReport)
%ASSEMBLELBTSRESULT 回溯 belief 父指针并拼接可直接执行/绘图的数据。

result=struct('success',success,'cost',inf,'vertexPath',[], ...
    'beliefIndexPath',[],'edgePath',zeros(0,2),'nominalTrajectory',[], ...
    'controlTrajectory',[],'time',[],'covarianceSequence',{{}}, ...
    'estimationCovarianceSequence',{{}},'edgeRisks',[], ...
    'feedbackGains',{{}},'kalmanGains',{{}}, ...
    'linearizationA',{{}},'linearizationB',{{}}, ...
    'measurementMatrices',{{}},'measurementCovariances',{{}}, ...
    'edgeStepCounts',[], ...
    'beliefs',[],'heuristic',heuristic,'statistics',statistics, ...
    'options',options,'startIndex',startIndex,'goalIndex',goalIndex, ...
    'attachmentReport',attachmentReport,'trajectoryGraph',[], ...
    'world',graph.param.world,'model',graph.param.model);
if options.output.includeBeliefs, result.beliefs=beliefs; end
if options.output.includeTrajectoryGraph, result.trajectoryGraph=graph; end
if ~success, return; end

ids=zeros(0,2); current=goalId;
while all(current>0)
    ids=[current;ids]; %#ok<AGROW>
    node=beliefs{current(1)}(current(2));
    current=node.parent;
end
result.vertexPath=ids(:,1)';
result.beliefIndexPath=ids(:,2)';
result.cost=beliefs{goalId(1)}(goalId(2)).costToCome;
if size(ids,1)==1
    result.nominalTrajectory=graph.Vertices(ids(1,1),:)';
    result.time=0;
    result.covarianceSequence={beliefs{ids(1,1)}(ids(1,2)).P};
    result.estimationCovarianceSequence={beliefs{ids(1,1)}(ids(1,2)).Ptilde};
    return
end

X=[]; U=[]; t=[]; Pseq={}; Ptseq={}; risks=[];
Kseq={}; Lseq={}; Aseq={}; Bseq={}; Cseq={}; Rseq={}; edgeSteps=[];
for q=2:size(ids,1)
    source=ids(q-1,1); target=ids(q,1);
    node=beliefs{target}(ids(q,2));
    edgeIndex=find(graph.Edges{source}==target,1,'first');
    edge=graph.EdgeData{source}{edgeIndex};
    xedge=edge{1}; uedge=edge{3};
    localTime=node.incoming.time;
    if isempty(X)
        X=xedge; U=uedge; t=localTime;
        Pseq=node.incoming.PSequence;
        Ptseq=node.incoming.PtildeSequence;
    else
        X=[X,xedge(:,2:end)]; %#ok<AGROW>
        U=[U,uedge]; %#ok<AGROW>
        t=[t,t(end)+localTime(2:end)]; %#ok<AGROW>
        Pseq=[Pseq,node.incoming.PSequence(2:end)]; %#ok<AGROW>
        Ptseq=[Ptseq,node.incoming.PtildeSequence(2:end)]; %#ok<AGROW>
    end
    risks(end+1)=node.incoming.risk; %#ok<AGROW>
    edgeSteps(end+1)=numel(node.incoming.gains); %#ok<AGROW>
    Kseq=[Kseq,node.incoming.gains]; %#ok<AGROW>
    Lseq=[Lseq,node.incoming.kalmanGains]; %#ok<AGROW>
    Aseq=[Aseq,node.incoming.A]; %#ok<AGROW>
    Bseq=[Bseq,node.incoming.B]; %#ok<AGROW>
    Cseq=[Cseq,node.incoming.measurementMatrices]; %#ok<AGROW>
    Rseq=[Rseq,node.incoming.measurementCovariances]; %#ok<AGROW>
end
result.edgePath=[ids(1:end-1,1),ids(2:end,1)];
result.nominalTrajectory=X;
result.controlTrajectory=U;
result.time=t;
result.covarianceSequence=Pseq;
result.estimationCovarianceSequence=Ptseq;
result.edgeRisks=risks;
result.feedbackGains=Kseq;
result.kalmanGains=Lseq;
result.linearizationA=Aseq;
result.linearizationB=Bseq;
result.measurementMatrices=Cseq;
result.measurementCovariances=Rseq;
result.edgeStepCounts=edgeSteps;
end
