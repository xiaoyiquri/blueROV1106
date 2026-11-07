function roadmap = buildKinodynamicPRM(param)
%BUILDKINODYNAMICPRM Build a directed kinodynamic PRM for BlueROV2 Heavy.
%
% EdgeData{i}{k} 与 Edges{i}(k) 对齐，并保存：
%   {stateTrajectory, N, thrusterForces, solverInfo}
%
% 算法流程：
%   1) 在三维自由空间采样位置，并扩展成 12 维状态节点；
%   2) 对每个节点选择位置半径内的全部节点，不排序、不截取固定数量；
%   3) 在 NED 坐标系检查起点和终点速度方向，再调用 meanControl；
%   4) 只要 NLP 成功就加入边；离线阶段完全不检查障碍物；
%   5) 保存邻接表、反向父表、代价和完整边轨迹。
%
% 离线路网仅描述 BlueROV2 的动力学可达性，因此与具体障碍物地图无关。
% 在线查询时，queryKinodynamicPRM 会根据当前 world 检查每条候选轨迹，
% 构造临时无碰撞邻接表，再在该子图上运行 Dijkstra。

if nargin < 1 || isempty(param)
    param = bluerov2Parameters();
end
[Vertices, positions] = sampleBlueROV2States(param);
n = size(Vertices, 1);
% EdgeData 与邻接表同序，空间复杂度为 O(V+E)，不会分配 n x n cell。
Edges = cell(n, 1);
Parents = cell(n, 1);
EdgesCost = cell(n, 1);
EdgeData = cell(n, 1);
for i=1:n, EdgeData{i}=cell(1,0); end
edgeCount = 0;
attemptCount = 0;

fprintf('Building BlueROV2 kinodynamic PRM with %d states...\n', n);
for i = 1:n
    % 与原 graph_heuristic.m 的 Adding edges 一致，只按位置欧氏距离
    % 选择半径内的全部候选。下界用于排除同一位置的不同速度状态。
    positionDistance = vecnorm(Vertices(:, 1:3)-Vertices(i, 1:3), 2, 2);
    candidates = find(positionDistance < param.prm.connectionRadius & ...
        positionDistance >= param.prm.minEdgePositionDistance);
    if isempty(candidates)
        continue
    end

    for c = 1:numel(candidates)
        j = candidates(c);

        % SampleMultipleVel 型采样使每个状态的线速度指向某个邻点。
        % 仅当 i 的惯性系速度指向 j，且 j 的速度不沿来向倒退时尝试
        % 局部 OCP。该三维点积判据能够区分同向和反向，比二维共线
        % 判据更适合 BlueROV2 的六自由度状态。
        if ~isDirectionCompatible(Vertices(i, :), Vertices(j, :), param.prm)
            continue
        end
        if any(Edges{i}==j)
            continue
        end

        % 图是有向的，此处只求解 i->j。反向 j->i 会在外层循环到达
        % j 时独立接受速度方向检查，不能在这里无条件强制求解。
        attemptCount = attemptCount + 1;
        [ok, edge] = solveLocalEdge(Vertices(i, :)', Vertices(j, :)', param);
        if ok
            [Edges, Parents, EdgesCost, EdgeData] = insertEdge( ...
                Edges, Parents, EdgesCost, EdgeData, i, j, edge);
            edgeCount = edgeCount + 1;
        end
    end
    if mod(i, max(1, floor(n/10))) == 0
        fprintf('  %3.0f%%, accepted %d / attempted %d directed edges\n', ...
            100*i/n, edgeCount, attemptCount);
    end
end

roadmap=struct();
roadmap.Vertices=Vertices;
roadmap.Edges=Edges;
roadmap.Parents=Parents;
roadmap.EdgesCost=EdgesCost;
roadmap.EdgeData=EdgeData;
roadmap.param=param;
roadmap.edge_count=edgeCount;
roadmap.attempt_count=attemptCount;
roadmap.sampled_positions=positions;

if isfield(param.prm, 'saveFile') && ~isempty(param.prm.saveFile)
    saveRoadmap(param.prm.saveFile, roadmap);
end
fprintf('Finished: %d feasible directed edges from %d attempts.\n', ...
    edgeCount, attemptCount);
end

function compatible = isDirectionCompatible(xFrom, xTo, prm)
%ISDIRECTIONCOMPATIBLE 判断候选有向边的三维线速度方向是否相容。
%
% 状态中的 [u v w] 是机体系速度，不能直接与 NED 坐标中的位置差
% 作点积。因此先用各节点姿态对应的 ZYX 旋转矩阵将速度转换到 NED，
% 再用归一化点积计算速度与边方向的方向余弦。
deltaPosition = xTo(1:3)-xFrom(1:3);
distance = norm(deltaPosition);
if distance < prm.minEdgePositionDistance
    compatible = false;
    return
end
edgeDirection = deltaPosition(:)/distance;

RFrom = rotationZYXNumeric(xFrom(4), xFrom(5), xFrom(6));
RTo = rotationZYXNumeric(xTo(4), xTo(5), xTo(6));
velocityFromNed = RFrom*xFrom(7:9)';
velocityToNed = RTo*xTo(7:9)';
speedFrom = norm(velocityFromNed);
speedTo = norm(velocityToNed);

% 当前离线路网不采样零速度状态。这里仍保留零速度兼容分支，使函数
% 对用户自定义节点更稳健；静止查询端点则由在线连接代码单独处理。
if speedFrom <= prm.directionSpeedTolerance
    sourceCompatible = true;
else
    sourceCosine = dot(velocityFromNed, edgeDirection)/speedFrom;
    sourceCompatible = sourceCosine >= prm.sourceDirectionCosine;
end
if speedTo <= prm.directionSpeedTolerance
    targetCompatible = true;
else
    targetCosine = dot(velocityToNed, edgeDirection)/speedTo;
    targetCompatible = targetCosine >= prm.targetDirectionCosine;
end
compatible = sourceCompatible && targetCompatible;
end

function R = rotationZYXNumeric(phi, theta, psi)
%ROTATIONZYXNUMERIC 从机体系到 NED 的 ZYX 欧拉角旋转矩阵。
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi; ...
     cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi; ...
     -sth, sphi*cth, cphi*cth];
end

function [ok, edge] = solveLocalEdge(x0, xf, param)
% 离线边只表达动力学可达性。这里故意不读取 world.obstacles，也不做
% 轨迹碰撞检查，从而保证生成的 roadmap 能在不同障碍物场景中复用。
[xbar, cost, N, ubar, info] = meanControl(x0, xf, param);
ok = info.success && isfinite(cost);
edge = {xbar, N, ubar, info};
if ~ok
    edge = [];
end
end

function [Edges, Parents, Costs, Data] = insertEdge( ...
        Edges, Parents, Costs, Data, i, j, edge)
% 四个容器必须以完全相同的边顺序更新：
%   Edges{i}(k)=j 对应 Costs{i}(k)，而 Parents{j} 用于反向遍历。
Edges{i}(end+1) = j;
Parents{j}(end+1) = i;
Costs{i}(end+1) = edgeCost(edge);
Data{i}{end+1} = edge;
end

function c = edgeCost(edge)
% The exact objective returned by meanControl is stored in solverInfo.
info = edge{4};
if isempty(edge{3}) || ~isfinite(info.T)
    c = inf;
    return
end
c = info.cost;
end

function saveRoadmap(fileName, roadmap)
% 只保存单份规范结构，避免 trajectory data 在同一 MAT 中重复。
saveTrajectoryGraph(fileName,roadmap);
end
