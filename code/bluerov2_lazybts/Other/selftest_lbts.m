function selftest_lbts()
%SELFTEST_LBTS 不调用 IPOPT 的轻量级接口与数值单元测试。
% 完整端到端测试仍需用户的 trajectoryGraph.mat、CasADi 和 MATLAB。

fprintf('Running BlueROV2 Lazy BTS helper tests...\n');
P=nearestPSD([1 0;0 -1e-12],0);
assert(min(eig(P))>=-1e-14);

graph.Vertices=zeros(3,12);
graph.Edges={[2 3];3;[]};
graph.EdgesCost={[1 5];2;[]};
h=backwardDijkstra(graph,3);
assert(norm(h-[3;2;0])<1e-12);

defaults=defaultLBTSOptions();
world=struct('positionBounds',[-2 2;-2 2;0 4], ...
    'robotRadius',.2,'collisionStep',.1,'obstacles',struct([]));
assert(isPositionCollisionFreeLBTS([0;0;2],world));
assert(~isPositionCollisionFreeLBTS([1.9;0;2],world));

[C,R]=measurementModelAtState(zeros(12,1),defaults.sensor);
assert(all(size(C)==[12 12]) && all(size(R)==[12 12]));
Z=standardGaussianSamples(3,101,7);
assert(all(size(Z)==[3 101]) && norm(mean(Z(:,1:100),2))<1e-12);
fprintf('All helper tests passed.\n');
end
