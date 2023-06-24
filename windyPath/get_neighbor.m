function [neighbor] = get_neighbor(xcurrent,ycurrent,zcurrent)
offsets = [0,0,1;0,1,0;0,1,1;1,0,0;1,0,1;1,1,0;1,1,1;];
for i=1:size(offsets,1)
    xneighbor=xcurrent+offsets(i,1);
    yneighbor=ycurrent+offsets(i,2);
    zneighbor=zcurrent+offsets(i,3);
end
end

