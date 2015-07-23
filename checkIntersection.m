
%% treats cubes who share boundaries as a non-empty intersection
%TBD: VECTORIZE
function result = checkIntersection(cube1,cube2)

I = [max(cube1(:,1),cube2(:,1)) min(cube1(:,2),cube2(:,2))];
diff = I(:,2) - I(:,1);
% diff = fix(diff*1e8)/1e8;
result = all(diff >= 0);
% result = all((I(:,2) - I(:,1))>=0);
end
