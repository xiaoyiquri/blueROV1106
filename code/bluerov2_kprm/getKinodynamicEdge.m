function [edge, edgeIndex] = getKinodynamicEdge(roadmap, source, target)
%GETKINODYNAMICEDGE 按有向端点读取紧凑 EdgeData 中的一条轨迹。
edgeIndex=find(roadmap.Edges{source}==target,1,'first');
if isempty(edgeIndex)
    edge=[];
else
    edge=roadmap.EdgeData{source}{edgeIndex};
end
end
