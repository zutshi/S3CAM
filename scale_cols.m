% TODO: Do not use with sparse matrices
function y = scale_cols(x, s)
y = x.*repmat(s(:).', rows(x), 1);
end