function graph=loadTrajectoryGraph(fileName)
%LOADTRAJECTORYGRAPH 加载当前 KPRM 生成的 trajectoryGraph.mat。
%
% 文件必须包含 trajectoryGraph 结构体，边数据采用：
%   EdgeData{i}{k} 对应 Edges{i}(k) 和 EdgesCost{i}(k)。

if nargin<1 || isempty(fileName), fileName='trajectoryGraph.mat'; end
assert(isfile(fileName),'LBTS:GraphNotFound','找不到 trajectoryGraph 文件：%s',fileName);
raw=load(fileName,'trajectoryGraph');
assert(isfield(raw,'trajectoryGraph') && isstruct(raw.trajectoryGraph),'LBTS:InvalidGraph','MAT 文件必须包含 trajectoryGraph 结构体。');
graph=raw.trajectoryGraph;

required={'Vertices','Edges','Parents','EdgesCost','EdgeData','param'};
for k=1:numel(required)
    assert(isfield(graph,required{k}),'LBTS:InvalidGraph','trajectoryGraph 缺少字段 %s。',required{k});
end
assert(isnumeric(graph.Vertices) && size(graph.Vertices,2)==12,'LBTS:InvalidStateDimension','Vertices 必须是 n x 12 数值矩阵。');
n=size(graph.Vertices,1);
graph.Edges=reshape(graph.Edges,[],1);
graph.Parents=reshape(graph.Parents,[],1);
graph.EdgesCost=reshape(graph.EdgesCost,[],1);
graph.EdgeData=reshape(graph.EdgeData,[],1);
assert(numel(graph.Edges)==n && numel(graph.Parents)==n && numel(graph.EdgesCost)==n && numel(graph.EdgeData)==n,'LBTS:InvalidGraph','图的 cell 数量必须等于顶点数。');

for i=1:n
    graph.Edges{i}=reshape(graph.Edges{i},1,[]);
    graph.Parents{i}=reshape(graph.Parents{i},1,[]);
    graph.EdgesCost{i}=reshape(graph.EdgesCost{i},1,[]);
    graph.EdgeData{i}=reshape(graph.EdgeData{i},1,[]);
    degree=numel(graph.Edges{i});
    assert(numel(graph.EdgesCost{i})==degree && numel(graph.EdgeData{i})==degree,'LBTS:InvalidGraph','顶点 %d 的 Edges、EdgesCost 和 EdgeData 长度不同。',i);
end
graph.sourceFile=char(fileName);
end
