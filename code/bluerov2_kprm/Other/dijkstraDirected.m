function [path,totalCost,distance]=dijkstraDirected(Edges,EdgesCost,startIdx,goalIdx)
%DIJKSTRADIRECTED 用二叉最小堆求非负权有向图最短路。
%
% 堆中允许同一顶点存在旧条目；弹出时以 distance 跳过过期条目。这种
% lazy decrease-key 实现保持接口简单，复杂度由数组扫描的 O(V^2)
% 降为 O((V+E)log E)，对扩大后的 trajectoryGraph 更稳定。
n=numel(Edges);
distance=inf(n,1); previous=zeros(n,1); visited=false(n,1);
distance(startIdx)=0;
heapNodes=zeros(1,max(16,n)); heapCosts=inf(size(heapNodes)); heapSize=0;
[heapNodes,heapCosts,heapSize]=heapPush( ...
    heapNodes,heapCosts,heapSize,startIdx,0);

while heapSize>0
    [u,best,heapNodes,heapCosts,heapSize]=heapPop( ...
        heapNodes,heapCosts,heapSize);
    if visited(u) || best>distance(u), continue; end
    visited(u)=true;
    if u==goalIdx, break; end
    children=Edges{u}; costs=EdgesCost{u};
    for k=1:numel(children)
        v=children(k); c=costs(k);
        if ~isfinite(c) || c<0, continue; end
        alternative=best+c;
        if alternative<distance(v)
            distance(v)=alternative;
            previous(v)=u;
            [heapNodes,heapCosts,heapSize]=heapPush( ...
                heapNodes,heapCosts,heapSize,v,alternative);
        end
    end
end

totalCost=distance(goalIdx);
if ~isfinite(totalCost), path=[]; return; end
reversePath=zeros(1,n); count=1; reversePath(count)=goalIdx;
while reversePath(count)~=startIdx
    parent=previous(reversePath(count));
    if parent==0, path=[]; totalCost=inf; return; end
    count=count+1; reversePath(count)=parent;
end
path=reversePath(count:-1:1);
end

function [nodes,costs,count]=heapPush(nodes,costs,count,node,cost)
count=count+1;
if count>numel(nodes)
    nodes=[nodes,zeros(size(nodes))];
    costs=[costs,inf(size(costs))];
end
q=count;
while q>1
    parent=floor(q/2);
    if costs(parent)<=cost, break; end
    nodes(q)=nodes(parent); costs(q)=costs(parent); q=parent;
end
nodes(q)=node; costs(q)=cost;
end

function [node,cost,nodes,costs,count]=heapPop(nodes,costs,count)
node=nodes(1); cost=costs(1);
lastNode=nodes(count); lastCost=costs(count); count=count-1;
if count==0, return; end
q=1;
while true
    left=2*q;
    if left>count, break; end
    right=left+1; child=left;
    if right<=count && costs(right)<costs(left), child=right; end
    if costs(child)>=lastCost, break; end
    nodes(q)=nodes(child); costs(q)=costs(child); q=child;
end
nodes(q)=lastNode; costs(q)=lastCost;
end
