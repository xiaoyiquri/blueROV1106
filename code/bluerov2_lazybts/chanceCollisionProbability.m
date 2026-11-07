function [maximumRisk, riskByStep, sampleHistory] = ...
        chanceCollisionProbability(xbar, covarianceSequence, world, Z, ...
        includeNominalPoint, plotSettings)
%CHANCECOLLISIONPROBABILITY 逐时刻估计碰撞概率，并可视化三维分布。
%
% 对第 k 个离散时刻，从三维位置边缘分布
%   p_k ~ N(xbar(1:3,k), P_k(1:3,1:3))
% 产生样本并做包围球碰撞检测。返回所有时刻中最大的样本碰撞比例。
% 这是离散的“逐时刻机会约束”，不是整条路径联合碰撞概率。
%
% 可视化默认关闭，以免 LazyBTS 搜索每扩展一条边都刷新窗口。开启方法：
%   options.chance.visualize = true;
%   options.chance.plotMode = 'animate';   % 或 'accumulate'
% LBTS 会通过 propagateBelief 把 options.chance 作为 plotSettings 传入。
%
% 图中含义：
%   蓝点  当前时刻的无碰撞高斯样本；
%   红点  按机器人包围球检测后发生碰撞的样本；
%   青色椭球 位置协方差的 covarianceSigma-sigma 等值面；
%   灰色实体 障碍物；红色透明外表面是考虑 robotRadius 后的禁入区域。
%
% 第三个输出 sampleHistory{k} 保存 samples 和 collisionMask，便于在不
% 画图时复查单个时刻。只有请求第三输出或开启可视化时才保留该数据。

if nargin < 6 || isempty(plotSettings)
    plotSettings = struct();
end
plotSettings = completePlotSettings(plotSettings);

assert(size(xbar,1)>=3, 'xbar 至少必须包含三维位置。');
K = size(xbar,2);
assert(numel(covarianceSequence)==K, ...
    'covarianceSequence 的元素数必须与 xbar 的时刻数相同。');
assert(size(Z,1)==3 && size(Z,2)>=1, 'Z 必须是 3 x numSamples。');

riskByStep = nan(1,K);
keepSamples = nargout>=3 || plotSettings.visualize;
if keepSamples
    sampleHistory = cell(1,K);
else
    sampleHistory = {};
end

plotContext = [];
if plotSettings.visualize
    plotContext = initializeChancePlot(world,xbar,plotSettings);
end

for k = 1:K
    % 仅取完整 12 维状态协方差的位置子块。位置椭球能在三维工作空间中
    % 直接显示；姿态和速度协方差无法与 N/E/D 共用同一组三维坐标轴。
    Ppos = nearestPSD(covarianceSequence{k}(1:3,1:3), 0);
    % 不额外添加 jitter：椭球和样本必须严格使用同一个 Ppos。若 Ppos
    % 只有半正定性，下面的特征分解会在零方差方向生成严格为零的扰动。
    [L, flag] = chol(Ppos, 'lower');
    if flag ~= 0
        % 半正定但非正定（例如某方向方差为零）时，用特征分解平方根。
        [V,D] = eig(Ppos);
        L = V*diag(sqrt(max(real(diag(D)),0)));
    end
    samples = xbar(1:3,k) + L*Z;
    % 一次处理全部样本：边界、球体和轴对齐盒体都可按列广播计算，
    % 避免 timestep x sample x obstacle 的 MATLAB 标量函数调用。
    collisionMask = collisionMaskForSamples(samples,world);
    riskByStep(k) = nnz(collisionMask)/size(samples,2);
    nominalCollision = ~isPositionCollisionFreeLBTS(xbar(1:3,k),world);
    if includeNominalPoint && nominalCollision
        riskByStep(k) = 1;
    end

    if keepSamples
        % Mahalanobis 覆盖率只服务于绘图/第三输出；正常规划无需计算。
        [insideEllipsoidMask,theoreticalCoverage] = ellipsoidMembership( ...
            samples,xbar(1:3,k),Ppos,plotSettings.covarianceSigma);
        sampleHistory{k} = struct('mean',xbar(1:3,k), ...
            'positionCovariance',Ppos,'samples',samples, ...
            'collisionMask',collisionMask, ...
            'sampleCollisionProbability',nnz(collisionMask)/size(samples,2), ...
            'ellipsoidSigma',plotSettings.covarianceSigma, ...
            'insideEllipsoidMask',insideEllipsoidMask, ...
            'empiricalEllipsoidCoverage',mean(insideEllipsoidMask), ...
            'theoreticalEllipsoidCoverage',theoreticalCoverage, ...
            'nominalCollision',nominalCollision);
    end
    if plotSettings.visualize
        plotContext = drawChanceStep(plotContext,xbar,Ppos,samples, ...
            collisionMask,riskByStep(k),insideEllipsoidMask, ...
            theoreticalCoverage,k,K,plotSettings);
    end
    if ~keepSamples && riskByStep(k)>plotSettings.stopThreshold
        % 搜索只关心是否超过阈值。拒绝已确定时不再计算后续时刻。
        break
    end
end
maximumRisk = max(riskByStep(1:k));
end

function settings = completePlotSettings(settings)
% 允许直接调用本函数时只给出少量绘图字段，不要求构造完整 options。
defaults = struct('visualize',false,'plotMode','animate', ...
    'pauseSeconds',0.05,'covarianceSigma',3.0,'sampleMarkerSize',16, ...
    'reuseFigure',true,'figureHandle',[], ...
    'showInflatedObstacles',true,'stopThreshold',inf);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(settings,names{k}) || isempty(settings.(names{k}))
        settings.(names{k}) = defaults.(names{k});
    end
end
settings.plotMode = lower(char(settings.plotMode));
assert(any(strcmp(settings.plotMode,{'animate','accumulate'})), ...
    'chance.plotMode 必须是 animate 或 accumulate。');
assert(settings.covarianceSigma>0, 'covarianceSigma 必须为正数。');
assert(settings.pauseSeconds>=0, 'pauseSeconds 不能为负数。');
end

function collisionMask=collisionMaskForSamples(samples,world)
%COLLISIONMASKFORSAMPLES 对 3xS 位置样本执行完全向量化的包围球检测。
r=world.robotRadius;
lowerBound=world.positionBounds(:,1)+r;
upperBound=world.positionBounds(:,2)-r;
collisionMask=any(samples<lowerBound | samples>upperBound,1);
for q=1:numel(world.obstacles)
    obstacle=world.obstacles(q);
    switch lower(obstacle.type)
        case 'sphere'
            delta=samples-obstacle.center(:);
            collisionMask=collisionMask | ...
                sum(delta.*delta,1)<=(obstacle.radius+r)^2;
        case 'box'
            halfSize=obstacle.size(:)/2+r;
            collisionMask=collisionMask | ...
                all(abs(samples-obstacle.center(:))<=halfSize,1);
        otherwise
            error('LBTS:UnknownObstacle','未知障碍物类型：%s',obstacle.type);
    end
end
end

function context = initializeChancePlot(world,xbar,settings)
% 每条被检查的轨迹只初始化一次 world；逐时刻刷新时保留静态障碍物。
tag = 'BlueROV2ChanceCollisionProbability';
if ~isempty(settings.figureHandle) && isgraphics(settings.figureHandle,'figure')
    fig = settings.figureHandle;
elseif settings.reuseFigure
    existing = findobj(groot,'Type','figure','Tag',tag);
    if isempty(existing)
        fig = figure('Color','w','Name','BlueROV2 chance-constraint check', ...
            'Tag',tag);
    else
        fig = existing(1);
    end
else
    fig = figure('Color','w','Name','BlueROV2 chance-constraint check', ...
        'Tag',tag);
end
clf(fig);
ax = axes('Parent',fig);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
xlabel(ax,'North [m]'); ylabel(ax,'East [m]'); zlabel(ax,'Depth [m]');
set(ax,'ZDir','reverse');
drawChanceWorld(ax,world,settings.showInflatedObstacles);
plot3(ax,xbar(1,:),xbar(2,:),xbar(3,:),'-', ...
    'Color',[.35 .35 .35],'LineWidth',1.2,'DisplayName','nominal trajectory');
% 用固定代理对象建立图例；animate 模式会删除每一步的动态对象，固定
% 图例可避免条目随刷新消失或重复。
plot3(ax,nan,nan,nan,'o','MarkerSize',8,'MarkerFaceColor',[.10 .75 .80], ...
    'MarkerEdgeColor',[.05 .45 .50], ...
    'DisplayName',sprintf('%.1f-sigma position covariance', ...
    settings.covarianceSigma));
scatter3(ax,nan,nan,nan,settings.sampleMarkerSize,[.10 .45 .95], ...
    'filled','DisplayName','collision-free sample');
scatter3(ax,nan,nan,nan,1.35*settings.sampleMarkerSize,[.90 .08 .08], ...
    'filled','MarkerEdgeColor',[.45 0 0],'DisplayName','collision sample');
scatter3(ax,nan,nan,nan,70,'k','filled','DisplayName','nominal state');
plot3(ax,nan,nan,nan,'s','MarkerSize',8,'MarkerFaceColor',[.35 .35 .35], ...
    'MarkerEdgeColor',[.35 .35 .35],'DisplayName','physical obstacle');
plot3(ax,nan,nan,nan,'s','MarkerSize',8,'MarkerFaceColor',[.95 .20 .15], ...
    'MarkerEdgeColor',[.80 .10 .08],'DisplayName','inflated forbidden region');
legend(ax,'Location','northeastoutside');
b = world.positionBounds;
xlim(ax,b(1,:)); ylim(ax,b(2,:)); zlim(ax,b(3,:));
context = struct('figure',fig,'axes',ax,'dynamicHandles',gobjects(0));
end

function context = drawChanceStep(context,xbar,Ppos,samples,collisionMask, ...
        risk,insideEllipsoidMask,theoreticalCoverage,k,K,settings)
ax = context.axes;
if strcmp(settings.plotMode,'animate')
    old = context.dynamicHandles;
    if ~isempty(old), delete(old(isgraphics(old))); end
    context.dynamicHandles = gobjects(0);
end

handles = gobjects(0);
% 已走过的均值轨迹用黑线强调；完整候选轨迹仍以灰线作为背景。
handles(end+1) = plot3(ax,xbar(1,1:k),xbar(2,1:k),xbar(3,1:k), ...
    'k-','LineWidth',1.8,'HandleVisibility','off');
handles(end+1) = drawPositionCovariance(ax,xbar(1:3,k),Ppos, ...
    settings.covarianceSigma);

freeMask = ~collisionMask;
if any(freeMask)
    handles(end+1) = scatter3(ax,samples(1,freeMask),samples(2,freeMask), ...
        samples(3,freeMask),settings.sampleMarkerSize,[.10 .45 .95], ...
        'filled','MarkerFaceAlpha',.50,'HandleVisibility','off');
end
if any(collisionMask)
    handles(end+1) = scatter3(ax,samples(1,collisionMask), ...
        samples(2,collisionMask),samples(3,collisionMask), ...
        1.35*settings.sampleMarkerSize,[.90 .08 .08], ...
        'filled','MarkerEdgeColor',[.45 0 0],'HandleVisibility','off');
end
handles(end+1) = scatter3(ax,xbar(1,k),xbar(2,k),xbar(3,k),70, ...
    'k','filled','MarkerEdgeColor','w','LineWidth',.8, ...
    'HandleVisibility','off');

if strcmp(settings.plotMode,'animate')
    context.dynamicHandles = handles;
end
title(ax,sprintf(['Step %d/%d: collision=%.3f; %.1f-sigma coverage=' ...
    '%.3f (theory %.3f); samples=%d'],k,K,risk, ...
    settings.covarianceSigma,mean(insideEllipsoidMask), ...
    theoreticalCoverage,size(samples,2)));
drawnow;
if settings.pauseSeconds>0, pause(settings.pauseSeconds); end
end

function [insideMask,theoreticalCoverage] = ellipsoidMembership( ...
        samples,center,P,sigmaScale)
% 用与绘图完全相同的 P 计算 Mahalanobis 半径，检查样本是否位于椭球内。
% 对秩亏协方差，只在非零方差子空间计算自由度和距离。
[V,D] = eig(nearestPSD(P,0));
eigenvalues = max(real(diag(D)),0);
tolerance = max(1,max(eigenvalues))*1e-12;
active = eigenvalues>tolerance;
if ~any(active)
    insideMask = vecnorm(samples-center(:),2,1)<=sqrt(tolerance);
    theoreticalCoverage = 1;
    return
end
coordinates = V(:,active)'*(samples-center(:));
coordinates = coordinates./sqrt(eigenvalues(active));
mahalanobisSquared = sum(coordinates.^2,1);
insideMask = mahalanobisSquared<=sigmaScale^2*(1+1e-12);
effectiveDimension = nnz(active);
% Mahalanobis 距离平方服从自由度为有效维数的 chi-square 分布。
theoreticalCoverage = gammainc(sigmaScale^2/2,effectiveDimension/2);
end

function handle = drawPositionCovariance(ax,center,P,sigmaScale)
% 绘制 (p-mu)'P^dagger(p-mu)=sigmaScale^2 的三维等值面。
[V,D] = eig(nearestPSD(P,0));
radii = sigmaScale*sqrt(max(real(diag(D)),0));
[sx,sy,sz] = sphere(14);
points = V*diag(radii)*[sx(:)';sy(:)';sz(:)']+center(:);
sx = reshape(points(1,:),size(sx));
sy = reshape(points(2,:),size(sy));
sz = reshape(points(3,:),size(sz));
handle = surf(ax,sx,sy,sz,'FaceColor',[.10 .75 .80], ...
    'FaceAlpha',.20,'EdgeColor',[.05 .45 .50],'EdgeAlpha',.18, ...
    'HandleVisibility','off');
end

function drawChanceWorld(ax,world,showInflated)
% 画物理 world；红色外轮廓对应“机器人中心不得进入”的膨胀几何。
r = world.robotRadius;
b = world.positionBounds;
drawAxisAlignedBox(ax,mean(b,2),b(:,2)-b(:,1),[.05 .05 .05],0,'-');
if showInflated
    safeBounds = [b(:,1)+r,b(:,2)-r];
    if all(safeBounds(:,2)>safeBounds(:,1))
        drawAxisAlignedBox(ax,mean(safeBounds,2), ...
            safeBounds(:,2)-safeBounds(:,1),[.10 .55 .15],0,'--');
    end
end

for q = 1:numel(world.obstacles)
    obstacle = world.obstacles(q);
    switch lower(obstacle.type)
        case 'sphere'
            [sx,sy,sz] = sphere(24);
            surf(ax,obstacle.center(1)+obstacle.radius*sx, ...
                obstacle.center(2)+obstacle.radius*sy, ...
                obstacle.center(3)+obstacle.radius*sz, ...
                'FaceColor',[.35 .35 .35],'FaceAlpha',.55, ...
                'EdgeColor','none','HandleVisibility','off');
            if showInflated
                inflated = obstacle.radius+r;
                surf(ax,obstacle.center(1)+inflated*sx, ...
                    obstacle.center(2)+inflated*sy, ...
                    obstacle.center(3)+inflated*sz, ...
                    'FaceColor',[.95 .20 .15],'FaceAlpha',.06, ...
                    'EdgeColor',[.80 .10 .08],'EdgeAlpha',.20, ...
                    'HandleVisibility','off');
            end
        case 'box'
            drawSolidBox(ax,obstacle.center(:),obstacle.size(:), ...
                [.35 .35 .35],.55);
            if showInflated
                drawSolidBox(ax,obstacle.center(:),obstacle.size(:)+2*r, ...
                    [.95 .20 .15],.06);
            end
        otherwise
            error('LBTS:UnknownObstacle','未知障碍物类型：%s',obstacle.type);
    end
end
end

function handle = drawSolidBox(ax,center,boxSize,color,alpha)
[vertices,faces] = boxGeometry(center,boxSize);
handle = patch(ax,'Vertices',vertices,'Faces',faces,'FaceColor',color, ...
    'FaceAlpha',alpha,'EdgeColor',color,'HandleVisibility','off');
end

function handle = drawAxisAlignedBox(ax,center,boxSize,color,alpha,lineStyle)
[vertices,faces] = boxGeometry(center,boxSize);
handle = patch(ax,'Vertices',vertices,'Faces',faces,'FaceColor',color, ...
    'FaceAlpha',alpha,'EdgeColor',color,'LineStyle',lineStyle, ...
    'LineWidth',1.0,'HandleVisibility','off');
end

function [vertices,faces] = boxGeometry(center,boxSize)
c = center(:); h = boxSize(:)/2;
vertices = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
            -1 -1  1;1 -1  1;1 1  1;-1 1  1].*h'+c';
faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6; ...
         3 4 8 7;4 1 5 8];
end
