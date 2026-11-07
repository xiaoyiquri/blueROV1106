function roadmap = loadKinodynamicRoadmap(fileName)
%LOADKINODYNAMICROADMAP 加载新旧 KPRM MAT 布局并转为紧凑格式。
names=who('-file',fileName);
if ismember('trajectoryGraph',names)
    raw=load(fileName,'trajectoryGraph');
    roadmap=raw.trajectoryGraph;
elseif ismember('roadmap',names)
    raw=load(fileName,'roadmap');
    roadmap=raw.roadmap;
else
    required={'Vertices','Edges','EdgesCost'};
    assert(all(ismember(required,names)), ...
        'MAT 文件缺少 trajectoryGraph/roadmap 或基本图字段。');
    fields=required;
    optional={'Parents','EdgeData','Edges_data','param','edge_count', ...
        'attempt_count','sampled_positions'};
    fields=[fields,optional(ismember(optional,names))];
    raw=load(fileName,fields{:});
    roadmap=raw;
end
roadmap=normalizeKinodynamicRoadmap(roadmap);
roadmap.sourceFile=char(fileName);
end
