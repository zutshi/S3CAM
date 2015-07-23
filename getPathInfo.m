
function [d,p] = getPathInfo(path)
path(find(path==0)) = [];
d = path(end);
p = path(2:end-2);
end
