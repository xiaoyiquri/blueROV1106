function costToGo=backwardDijkstra(graph,goalIndex)
%BACKWARDDIJKSTRA 用二叉堆在反向名义图上计算可采纳 cost-to-go。
n=size(graph.Vertices,1);
incoming=cell(n,1); incomingCost=cell(n,1);
for i=1:n
    for k=1:numel(graph.Edges{i})
        j=graph.Edges{i}(k); c=graph.EdgesCost{i}(k);
        if isfinite(c) && c>=0
            incoming{j}(end+1)=i;
            incomingCost{j}(end+1)=c;
        end
    end
end
costToGo=inf(n,1); costToGo(goalIndex)=0; visited=false(n,1);
heapNodes=zeros(1,max(16,n)); heapCosts=inf(size(heapNodes)); heapSize=0;
[heapNodes,heapCosts,heapSize]=heapPush( ...
    heapNodes,heapCosts,heapSize,goalIndex,0);
while heapSize>0
    [j,best,heapNodes,heapCosts,heapSize]=heapPop( ...
        heapNodes,heapCosts,heapSize);
    if visited(j) || best>costToGo(j), continue; end
    visited(j)=true;
    for k=1:numel(incoming{j})
        i=incoming{j}(k);
        alternative=best+incomingCost{j}(k);
        if alternative<costToGo(i)
            costToGo(i)=alternative;
            [heapNodes,heapCosts,heapSize]=heapPush( ...
                heapNodes,heapCosts,heapSize,i,alternative);
        end
    end
end
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
