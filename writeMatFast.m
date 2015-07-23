
function [hdr,matStr] = writeMatFast(A,name,integer)
hdr = '';
matStr = '';
precision = '5';
delim = ' ';
if nargin == 4
    if strcmp(integer,'int') == 1
        formatStr = ['%d' delim];
    else
        formatStr = ['%.' precision 'f' delim];
    end
else
    formatStr = ['%.' precision 'f' delim];
end
[m,n] = size(A);
if ~isempty(name)
    hdr = sprintf('name: %s\n',name);
end

hdr = [hdr sprintf(['size: %d' delim '%d\n'],m,n)];
matStr = num2str(A,formatStr);
matStr(:,end+1) = ' ';
end
