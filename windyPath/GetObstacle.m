% function [obstacle,map]=GetObstacle(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle_number,obstacle)
% %�����ϰ��������
% %round����������Ϊ�����С����������rand�����ȷֲ����������
% map=zeros(mapsize);%mapsize��С��ȫ�����
% ob=round(rand([obstacle_number,3])*mapsize(3));%��������ϰ���
% removeIndex=[];
% node_start=[xstart,ystart,zstart];
% node_end=[xend,yend,zend];
% for io=1:length(ob(:,1))%����ob���飬�����Щ������start��goal�غϣ���������������removeInd��
%    if(isequal(ob(io,:),node_start) || isequal(ob(io,:),node_end))
%         removeIndex=[removeIndex;io];
%    end
% end
% ob(removeIndex,:)=[];%ɾ���ظ��Ľڵ�
% obstacle=[obstacle;ob];%��ob�ϰ�����뵽obstacle��
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
%��Բ���뾶
if rx<ry
    ellipse_radius=ry;
else
    ellipse_radius=rx;
end
%��Բ�е�
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

% �����ϰ��������
%      for i=25:30
%          for j=40:50
%              for k=1:45
%                  obstacle(size(obstacle,1)+1,:)=[i,j,k];
%              end
%          end
%      end
    
     
end