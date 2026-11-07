function dominated = isBeliefDominated(existingBeliefs, newCost, newP, ...
        newPtilde, settings)
%ISBELIEFDOMINATED 检查同一 trajectoryGraph 顶点上的 belief 偏序。
%
% 若已有节点同时满足
%   c_old <= c_new,
%   P_old <= P_new,
%   Ptilde_old <= Ptilde_new
%（矩阵不等式为 Loewner 半正定序），则新节点未来不可能优于旧节点，
% 可以安全丢弃。数值容差以 tol*I 加到差矩阵上。

dominated=false;
idx=settings.indices(:)';
I=eye(numel(idx));
for k=1:numel(existingBeliefs)
    old=existingBeliefs(k);
    if ~old.active || old.costToCome > newCost+settings.costTolerance
        continue
    end
    totalDifference=newP(idx,idx)-old.P(idx,idx)+settings.psdTolerance*I;
    estimationDifference=newPtilde(idx,idx)-old.Ptilde(idx,idx)+ ...
        settings.psdTolerance*I;
    if isPSD(totalDifference) && isPSD(estimationDifference)
        dominated=true;
        return
    end
end
end

function answer=isPSD(A)
A=(A+A')/2;
answer=min(eig(A))>=0;
end
