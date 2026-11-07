function result = queryKinodynamicPRM(roadmapOrFile, startState, goalState, currentWorld)
%QUERYKINODYNAMICPRM Backward-compatible one-call query wrapper.
%
% 新代码建议显式调用两步接口，以便在图搜索之前查看查询端点及新增边：
%   [queryRoadmap, queryInfo] = attachQueryEndpoints(...);
%   result = searchKinodynamicPRM(queryRoadmap, queryInfo, currentWorld);
% 本包装函数保留原来的单次调用方式。

if nargin < 4
    currentWorld = [];
end
[roadmap, queryInfo] = attachQueryEndpoints( ...
    roadmapOrFile, startState, goalState, currentWorld);
result = searchKinodynamicPRM(roadmap, queryInfo, currentWorld);
end
