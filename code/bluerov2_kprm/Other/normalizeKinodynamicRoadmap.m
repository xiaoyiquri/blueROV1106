function roadmap = normalizeKinodynamicRoadmap(roadmap)
%NORMALIZEKINODYNAMICROADMAP 统一为与邻接表同序的紧凑边数据格式。
%
% 新格式：
%   Edges{i}(k)       = 第 k 个后继顶点
%   EdgesCost{i}(k)   = 同一条边的标量代价
%   EdgeData{i}{k}    = {xbar,N,ubar,info}
%
% 本函数也接受旧的 n-by-n Edges_data{i,j}，并在内存中线性转换。转换后
% 删除旧字段，避免 96% 以上为空的稠密 cell 和真实轨迹同时占用内存。

required={'Vertices','Edges','EdgesCost'};
for q=1:numel(required)
    assert(isfield(roadmap,required{q}), ...
        'Roadmap 缺少字段 %s。',required{q});
end
n=size(roadmap.Vertices,1);
roadmap.Edges=reshape(roadmap.Edges,[],1);
roadmap.EdgesCost=reshape(roadmap.EdgesCost,[],1);
assert(numel(roadmap.Edges)==n && numel(roadmap.EdgesCost)==n, ...
    'Edges/EdgesCost 的 cell 数量必须等于顶点数。');

hasCompact=isfield(roadmap,'EdgeData') && iscell(roadmap.EdgeData);
hasLegacy=isfield(roadmap,'Edges_data') && iscell(roadmap.Edges_data);
if ~hasCompact && ~hasLegacy
    error('Roadmap 必须含 EdgeData 或旧式 Edges_data。');
end

compact=cell(n,1);
for i=1:n
    roadmap.Edges{i}=reshape(roadmap.Edges{i},1,[]);
    roadmap.EdgesCost{i}=reshape(roadmap.EdgesCost{i},1,[]);
    degree=numel(roadmap.Edges{i});
    assert(numel(roadmap.EdgesCost{i})==degree, ...
        '顶点 %d 的 Edges 与 EdgesCost 长度不同。',i);
    if hasCompact
        assert(numel(roadmap.EdgeData)>=i && iscell(roadmap.EdgeData{i}), ...
            'EdgeData{%d} 必须是 cell。',i);
        compact{i}=reshape(roadmap.EdgeData{i},1,[]);
        assert(numel(compact{i})==degree, ...
            '顶点 %d 的 EdgeData 与 Edges 长度不同。',i);
    else
        compact{i}=cell(1,degree);
        for k=1:degree
            compact{i}{k}=roadmap.Edges_data{i,roadmap.Edges{i}(k)};
        end
    end
end
roadmap.EdgeData=compact;
if isfield(roadmap,'Edges_data')
    roadmap=rmfield(roadmap,'Edges_data');
end

% Parents 是拓扑派生量，统一重建可同时修复旧文件中的顺序/遗漏。
roadmap.Parents=cell(n,1);
for i=1:n
    for j=roadmap.Edges{i}
        assert(j>=1 && j<=n && j==round(j), ...
            '顶点 %d 含非法后继索引。',i);
        roadmap.Parents{j}(end+1)=i;
    end
end
roadmap.edge_count=sum(cellfun(@numel,roadmap.Edges));
end
