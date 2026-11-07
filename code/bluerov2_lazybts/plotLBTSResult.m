function plotLBTSResult(result, world)
%PLOTLBTSRESULT 绘制三维名义路径、位置协方差椭球与规划环境。
%
% 椭球表示离散时刻位置边缘分布的 2-sigma 等值面，仅用于可视化；
% 真正的机会约束由 chanceCollisionProbability 中的包围球样本检测完成。

if ~result.success
    warning('LBTS:NoSolution','result.success=false，没有可绘制路径。');
    return
end
if nargin<2 || isempty(world)
    world=result.world;
end

figure('Color','w'); hold on; grid on; axis equal; view(3);
xlabel('North [m]'); ylabel('East [m]'); zlabel('Depth [m]');
title(sprintf('BlueROV2 Lazy BTS, cost = %.4g',result.cost));
set(gca,'ZDir','reverse');
drawWorld(world);
X=result.nominalTrajectory;
plot3(X(1,:),X(2,:),X(3,:),'b-','LineWidth',2.2);
plot3(X(1,1),X(2,1),X(3,1),'go','MarkerFaceColor','g','MarkerSize',8);
plot3(X(1,end),X(2,end),X(3,end),'rp','MarkerFaceColor','r','MarkerSize',10);

% 路径很长时只画至多约 20 个椭球，避免图形遮挡和渲染过慢。
stride=max(1,ceil(numel(result.covarianceSequence)/20));
for k=1:stride:numel(result.covarianceSequence)
    drawCovarianceEllipsoid(X(1:3,k), ...
        result.covarianceSequence{k}(1:3,1:3),2,[0.15 0.55 0.95]);
end
legend({'obstacle','nominal trajectory','start','goal'},'Location','best');
end

function drawWorld(world)
for k=1:numel(world.obstacles)
    obstacle=world.obstacles(k);
    switch lower(obstacle.type)
        case 'sphere'
            [x,y,z]=sphere(20);
            surf(obstacle.center(1)+obstacle.radius*x, ...
                obstacle.center(2)+obstacle.radius*y, ...
                obstacle.center(3)+obstacle.radius*z, ...
                'FaceColor',[.45 .45 .45],'FaceAlpha',.35,'EdgeColor','none');
        case 'box'
            c=obstacle.center(:); h=obstacle.size(:)/2;
            vertices=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
                      -1 -1 1;1 -1 1;1 1 1;-1 1 1].*h'+c';
            faces=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
            patch('Vertices',vertices,'Faces',faces,'FaceColor',[.45 .45 .45], ...
                'FaceAlpha',.35,'EdgeColor',[.25 .25 .25]);
    end
end
b=world.positionBounds;
xlim(b(1,:)); ylim(b(2,:)); zlim(b(3,:));
end

function drawCovarianceEllipsoid(center,P,scale,color)
P=nearestPSD(P,0);
[V,D]=eig(P); radii=scale*sqrt(max(real(diag(D)),0));
[x,y,z]=sphere(10);
points=V*diag(radii)*[x(:)';y(:)';z(:)']+center(:);
x=reshape(points(1,:),size(x)); y=reshape(points(2,:),size(y));
z=reshape(points(3,:),size(z));
surf(x,y,z,'FaceColor',color,'FaceAlpha',.12,'EdgeColor','none', ...
    'HandleVisibility','off');
end
