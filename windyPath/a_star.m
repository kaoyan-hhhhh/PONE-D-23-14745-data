%A���㷨������2D��
function [ path,route_node_number,open,close ] = a_star(xstart,ystart,xend,yend,mapsize,obstacle,ellipse_radius,ellipse_midpoint_x,ellipse_midpoint_y)

%��������(��ͼ)�Ĵ�С��x y z�ᣩ
x_size=mapsize(1);
y_size=mapsize(2);
%z_size=mapsize(3);

%���嵽���ھӵĽڵ�
%%��mapsizeΪ5*5*5�ľ�����zeros(mapsize)������5*5*5��ȫ�����
came_fromx=zeros(mapsize);
came_fromy=zeros(mapsize);
%came_fromz=zeros(mapsize);
came_fromx(xstart,ystart)=xstart;
came_fromy(xstart,ystart)=ystart;
%came_fromz(xstart,ystart,zstart)=zstart;

%����openlist����Žڵ�λ����Ϣ
open=[xstart,ystart];
%����closelist����Žڵ�λ����Ϣ
close=[];
%��������G��Fֵ��G:��㵽��ǰ�ڵ�ľ��롣F:��ǰ�ڵ㵽�յ�ľ��롣
%Inf*ones������mapsizeά����ͬ��һ������,�����ÿ��ֵ���������
G=Inf*ones(mapsize);
G(xstart,ystart)=0;
F=Inf*ones(mapsize);
F(xstart,ystart)=sqrt((xend-xstart)^2+(yend-ystart)^2);
% g(sstart)=0;
% parent(sstart)=sstart; 
% open.insert(start,g(sstart)+h(start));

%ʹ��append��������Ԫ�أ���openlist�б�������Ԫ��snode
%open=[xstart,ystart,zstart];

%��ʼ��
find_path=0;
gf=2;%��������

%��openlist�б���Ϊ��ʱ
     while isempty(open)==0 && find_path==0
     %����openlist��fx��С�Ľڵ�
     %����openlist��ÿһ���ڵ��fֵ
     %����fxlist��openlist��ÿһ���ڵ��fֵ������Сֵ������
     %��openlist��ȡ��fֵ��С�Ľڵ㣬��ֵ��cnode
             %��ʼ��������1��open��������*1���󣬴洢open�б���f��ֵ��size(open,1)����open��������
             %%B=zeros(m,n)������m��nȫ����
             %%size��A��������һ����ΪA����size��A�����ص���һ�����������������ĵ�һ��Ԫ��ʱ������������ڶ���Ԫ���Ǿ����������
             %%size(A,n)�������size���������������������һ��n������1��2Ϊn��ֵ���� size�����ؾ����������������
             %%����r=size(A,1)����䷵�ص�ʱ����A��������c=size(A,2)����䷵�ص�ʱ����A��������
             f_open=zeros(size(open,1),1);
             %��open�еĽڵ���fֵ��
             %%f_open(i,:)������f_open�е�i��Ԫ�ء�
             for i=1:size(open,1)
                    f_open(i,:)=F(open(i,1),open(i,2));
             end
             
             %����open��fֵ��С�Ľڵ������λ�á�~��ʾ���ԡ�
             %%[Y,U]=min(A)������������Y��U��Y������¼A��ÿ�е���Сֵ��U������¼ÿ����Сֵ���кš�
             [~,f_open_min_index]=min(f_open);
             %��open�б���ȡ��fֵ��С�Ľڵ�λ����Ϣ������ֵ����ǰ�ڵ�
             xcurrent=open(f_open_min_index,1);
             ycurrent=open(f_open_min_index,2);
%              zcurrent=open(f_open_min_index,3);
             current_node=[xcurrent, ycurrent];
             
%              %��openlist��fֵ��С�Ľڵ�ӡ������б�openlist����ɾ��
%              %%������������horzcat��
%              index_open_keep=horzcat(1:f_open_min_index-1,f_open_min_index+1:size(open,1));
%              open=open(index_open_keep,:);
             
             %����Ƿ񵽴�Ŀ��ڵ�
             if xcurrent==xend && ycurrent==yend
                   %����Ŀ��ڵ㣬�ҵ������·�����˳�������·��
                   find_path=1;
             else
                   %��openlist��fֵ��С�Ľڵ�ӡ������б�openlist����ɾ��
                   %%������������horzcat��
                   index_open_keep=horzcat(1:f_open_min_index-1,f_open_min_index+1:size(open,1));
                   open=open(index_open_keep,:);
                   %��openlist��ȡ��fֵ��С�Ľڵ�cnode���뵽���ر��б�closelist����
                   close(size(close,1)+1,:)=current_node;
                   
                   
     
                   % ��չ��ǰfx��С�Ľڵ㣬��������һ��ѭ������
                   % �����ھӽڵ�
                   for i=-1:1
                       for j=-1:1
%                            for k=-1:1

                                 
                                 %������ڽڵ���������
                                 if xcurrent+i>0 && ycurrent+j>0 && xcurrent+i<=x_size && ycurrent+j<=y_size
                                 
                                
                                     %����ھӽڵ�Ȳ�����openҲ������close
                                     %%��� A �Ǿ����� sum(A) �����ذ���ÿ���ܺ͵���������
                                     %%a(:,1)Ϊȡa�����е�һ��Ԫ�أ�a(1,:)Ϊȡa�����е�һ��Ԫ�ء�
                                     %%S = sum(A,dim) ��ά�� dim �����ܺ͡����磬��� A Ϊ������ sum(A,2) �ǰ���ÿһ���ܺ͵���������
                                     %%xcurrent+j==open(:,1)���� ��open��������*1 ����
                                     %check_open:����ھӽڵ��Ƿ���open�б��У�����3��˵����open�б��С�����С��3��˵������open�б��С�
                                     %check_closed������ھӽڵ��Ƿ���close�б��У�����3��˵����close�б��С�����С��3��˵������close�б��С�
                                     check_open=max(sum([xcurrent+i==open(:,1) ycurrent+j==open(:,2)],2));
                                     check_close=max(sum([xcurrent+i==close(:,1) ycurrent+j==close(:,2)],2));
                                     check_obstacle=max(sum([xcurrent+i==obstacle(:,1) ycurrent+j==obstacle(:,2)],2));

                                     %isemptyΪ��ʱ������1��isemptyΪ�ǿ�ʱ������0��
                                     if isempty(check_open)==1
                                         check_open=0;
                                     end

                                     if isempty(check_close)==1
                                         check_close=0;
                                     end

                                     if isempty(check_obstacle)==1
                                         check_obstacle=0;
                                     end
                                  
                                     
                                     
                                     
                                    
                                     %��2��д��
%                                      if check_close<3 && check_obstacle<3
%                                          if check_open<3
%                                              %��open�������ھӽڵ�node
%                                              %size(open,1)����open������
%                                              open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                          end
%                                          %����ӿ�ʼ����ǰ�ڵ�ľ���+��ǰ�ڵ㵽�ھӽڵ�ľ���
%                                          g_try=G(xcurrent,ycurrent,zcurrent)+sqrt(i^2+j^2+k^2);
% 
%                                          %����������С���ھӵľ���
%                                          if g_try<G(xcurrent+i,ycurrent+j,zcurrent+k)
% 
%                                               %����ȷ��·�ϣ�������Ϣ
% 
%                                               %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
%                                               came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=xcurrent; 
%                                               came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=ycurrent;
%                                               came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;
% 
%                                               %�����ɱ�����
%                                               G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                               H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                               %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
%                                               F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
%                                          end
%                                      end
                                         
                                             
                                       %��1��д��
                                     % �ж��ھӽڵ�node�Ƿ���closelist��blocklist��lowrisklist��
                                     if check_close==2 || check_obstacle==2
                                     % if check_close==3 
                                         continue
                                     else
                                         % ��if����жϵó����ڵ�node�Ȳ���closelist�У�Ҳ����blocklist��
                                         % ����ھӽڵ�node����openlist��
                                         if check_open<2
                                             %��open�������ھӽڵ�node
                                             %size(open,1)����open������
                                             open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j];
                                             %����ӿ�ʼ����ǰ�ڵ�ľ���+��ǰ�ڵ㵽�ھӽڵ�ľ���
                                             g_try=G(xcurrent,ycurrent)+sqrt(i^2+j^2);
                                             G(xcurrent+i,ycurrent+j)=g_try;
                                             H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2);
                                             F(xcurrent+i,ycurrent+j)=G(xcurrent+i,ycurrent+j)+H;
                                             
%                                              %���մ���
%                                              midpoint_distance_currentpoint=sqrt((ellipse_midpoint_x-(xcurrent+i))^2+(ellipse_midpoint_y-(ycurrent+j))^2);
%                                              if ellipse_radius<midpoint_distance_currentpoint
%                                                  R=0;
%                                              else
%                                                  R=sqrt(midpoint_distance_currentpoint^2+ellipse_radius^2);
%                                              end
%                                              F(xcurrent+i,ycurrent+j)=G(xcurrent+i,ycurrent+j)+H+R;
                                             
                                             %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
                                             came_fromx(xcurrent+i,ycurrent+j)=xcurrent; 
                                             came_fromy(xcurrent+i,ycurrent+j)=ycurrent;
%                                              came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;
                                         else
                                             % ������жϵó����ڵ�node����closelist�У�����blocklist�У���openlist��
              
                                             %����ӿ�ʼ����ǰ�ڵ�ľ���+��ǰ�ڵ㵽�ھӽڵ�ľ���
                                             g_try=G(xcurrent,ycurrent)+sqrt(i^2+j^2);

                                             %����������С���ھӵľ���
                                             if g_try<G(xcurrent+i,ycurrent+j)

                                                 %����ȷ��·�ϣ�������Ϣ

                                                 %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
                                                 came_fromx(xcurrent+i,ycurrent+j)=xcurrent; 
                                                 came_fromy(xcurrent+i,ycurrent+j)=ycurrent;
%                                                  came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;

                                                 %�����ɱ�����
                                                 G(xcurrent+i,ycurrent+j)=g_try;
                                                 H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2);
                                                 %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                                 F(xcurrent+i,ycurrent+j)=G(xcurrent+i,ycurrent+j)+H;
                                                 
%                                                  %���մ���
%                                                  midpoint_distance_currentpoint=sqrt((ellipse_midpoint_x-(xcurrent+i))^2+(ellipse_midpoint_y-(ycurrent+j))^2);
%                                                  if ellipse_radius<midpoint_distance_currentpoint
%                                                      R=0;
%                                                  else
%                                                      R=sqrt(midpoint_distance_currentpoint^2+ellipse_radius^2);
%                                                  end
%                                                  F(xcurrent+i,ycurrent+j)=G(xcurrent+i,ycurrent+j)+H+R;
                                                 end
                                         end
                                 

                                     end







                                 end
%                            end
                       end
                   end
             end
     end



     
     

%�����ؽ�·����֪�����ĸ��ڵ㵽����ھ�


%��һ��Ԫ���ǵ����
path_backwards=[xcurrent,ycurrent];

%��ʼ��
i=2;

%����㻹û�е���
while xcurrent~=xstart || ycurrent~=ystart 
    
    path_backwards(i,:)=[came_fromx(xcurrent,ycurrent) came_fromy(xcurrent,ycurrent)];
    xcurrent=path_backwards(i,1);
    ycurrent=path_backwards(i,2);    
%     zcurrent=path_backwards(i,3); 
    i=i+1;
    
end

%·���ڵ�����
route_node_number=size(path_backwards,1);


%����·������
path=path_backwards(route_node_number+1-(1:route_node_number),:);


%���·����ʼ�Ŀ�ʼ�ͽ�����
path(1,:)=[xstart ystart];
path(route_node_number,:)=[xend yend];
    


end
     
 

        
