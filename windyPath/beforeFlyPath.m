%发射之前的最短路径和时间
function [before_fly_path,route_node_number,pathLengthMin,pathTime] = beforeFlyPath(xstart,ystart,xend,yend)
% path1=[xstart,ystart-60;130,-30;440,-30;xend,yend-60];
% path2=[xstart,ystart-60;130,13;440,13;xend,yend-60];
% path1=[xstart,ystart-60;130,-40;440,-40;xend,yend-60];
% path2=[xstart,ystart-60;130,23;440,23;xend,yend-60];
path1=[xstart,ystart-60;120,-40;450,-40;xend,yend-60];
path2=[xstart,ystart-60;120,23;450,23;xend,yend-60];
route_node_number=4;
pathDistance1=zeros(route_node_number,1);
pathDistance2=zeros(route_node_number,1);
for i=2:route_node_number 
	pathDistance1(i)=pathDistance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
pathLength1=pathDistance1(route_node_number);
for i=2:route_node_number 
	pathDistance2(i)=pathDistance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
pathLength2=pathDistance2(route_node_number);
if pathLength1<pathLength2
    pathLengthMin=pathLength1;
    before_fly_path=path1;
else
    pathLengthMin=pathLength2;
    before_fly_path=path2;
end
pathTime=pathLengthMin/800;
end

