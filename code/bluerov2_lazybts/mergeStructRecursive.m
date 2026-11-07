function out = mergeStructRecursive(defaults, user)
%MERGESTRUCTRECURSIVE 递归覆盖参数，同时保留未指定的默认字段。
% 与简单 struct 赋值不同，本函数会继续合并嵌套结构，例如用户只设置
% options.search.verbose 时，不会意外删除 search.maxExpansions。

out = defaults;
if nargin < 2 || isempty(user)
    return
end
if ~isstruct(user)
    error('LBTS:InvalidOptions', 'options 必须是结构体。');
end
names = fieldnames(user);
for k = 1:numel(names)
    name = names{k};
    if isfield(out, name) && isstruct(out.(name)) && isstruct(user.(name)) && ...
            isscalar(out.(name)) && isscalar(user.(name))
        out.(name) = mergeStructRecursive(out.(name), user.(name));
    else
        out.(name) = user.(name);
    end
end
end
