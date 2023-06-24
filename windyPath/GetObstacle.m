% function [obstacle,map]=GetObstacle(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle_number,obstacle)
% %生成障碍点的坐标
% %round：四舍五入为最近的小数或整数。rand：均匀分布的随机数。
% map=zeros(mapsize);%mapsize大小的全零矩阵
% ob=round(rand([obstacle_number,3])*mapsize(3));%随机生产障碍点
% removeIndex=[];
% node_start=[xstart,ystart,zstart];
% node_end=[xend,yend,zend];
% for io=1:length(ob(:,1))%遍历ob数组，检查哪些坐标与start和goal重合，并将其索引存在removeInd中
%    if(isequal(ob(io,:),node_start) || isequal(ob(io,:),node_end))
%         removeIndex=[removeIndex;io];
%    end
% end
% ob(removeIndex,:)=[];%删除重复的节点
% obstacle=[obstacle;ob];%将ob障碍点加入到obstacle中
% map(obstacle(1),obstacle(2),obstacle(3))=1;
% 
% end

%%
function [obstacle,ellipse_radius,ellipse_midpoint_x,ellipse_midpoint_y] = GetObstacle()

obstacle=Cov_trajectory_point();
obstacle(:,2)=obstacle(:,2)+60;
obstacle=round(obstacle);

x_min=min(obstacle(:,1));
x_max=max(obstacle(:,1));
y_min=min(obstacle(:,2));
y_max=max(obstacle(:,2));
rx=abs(x_max-x_min)/2;
ry=abs(y_max-y_min)/2;
%椭圆长半径
if rx<ry
    ellipse_radius=ry;
else
    ellipse_radius=rx;
end
%椭圆中点
ellipse_midpoint_x=x_min+rx;
ellipse_midpoint_y=y_min+ry;



% x_min=min(obstacle(:,1));
% x_max=max(obstacle(:,1));
% y_min=min(obstacle(:,2));
% y_max=max(obstacle(:,2));
% z_min=min(obstacle(:,3));
% z_max=max(obstacle(:,3));
% for i=x_min:x_max
%     for j=y_min:y_max
%         for k=z_min:z_max
%             obstacle=[obstacle;i,j,k];
%         end
%     end
% end

% 生成障碍点的坐标
%      for i=25:30
%          for j=40:50
%              for k=1:45
%                  obstacle(size(obstacle,1)+1,:)=[i,j,k];
%              end
%          end
%      end
    
     
end