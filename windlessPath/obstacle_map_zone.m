function obstacle_map = obstacle_map_zone(mapsize,obstacle)
x_size=mapsize(1);
y_size=mapsize(2);
% z_size=mapsize(3);
obstacle_map=zeros(x_size,y_size);
% for i=1:x_size
%     for j=1:y_size
%         for z=1:z_size
%             obstacle_map(i,j,k)=1;
%         end
%     end
% end

%size(obstacle,1):返回obstacle有多少行
for i=1:size(obstacle,1)
    obstacle_map(obstacle(i,1),obstacle(i,2))=1;
end
    
end

