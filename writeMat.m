
% TODO: replace this with faster version
function writeMat(fid,A,name,integer)
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
    fprintf(fid,'name: %s\n',name);
end
fprintf(fid,['size: %d' delim '%d\n'],m,n);
for i = 1:m
    for j = 1:n
        fprintf(fid,formatStr,A(i,j));
    end
    %fprintf(fid,'\n');
end
fprintf(fid,'\n');
end
