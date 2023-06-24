%lazy theta算法函数
function [ path,route_node_number,open,close ] = lazy_theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,E3d_safe)

%环境矩阵(地图)的大小（x y z轴）
x_size=mapsize(1);
y_size=mapsize(2);
z_size=mapsize(3);

%定义到达邻居的节点
%%设mapsize为5*5*5的矩阵，则zeros(mapsize)会生成5*5*5的全零矩阵
came_fromx=zeros(mapsize);
came_fromy=zeros(mapsize);
came_fromz=zeros(mapsize);
came_fromx(xstart,ystart,zstart)=xstart;
came_fromy(xstart,ystart,zstart)=ystart;
came_fromz(xstart,ystart,zstart)=zstart;

%创建openlist，存放节点位置信息
%open=[xstart,ystart,zstart];
%创建closelist，存放节点位置信息
close=[];
%计算设置G和F值。G:起点到当前节点的距离。F:当前节点到终点的距离。
%Inf*ones生成与mapsize维数相同的一个矩阵,矩阵的每个值都是无穷大
G=Inf*ones(mapsize);
G(xstart,ystart,zstart)=0;
F=Inf*ones(mapsize);
F(xstart,ystart,zstart)=sqrt((xend-xstart)^2+(yend-ystart)^2+(zend-zstart)^2);
% g(sstart)=0;
% parent(sstart)=sstart; 
% open.insert(start,g(sstart)+h(start));

open=[xstart,ystart,zstart,F(xstart,ystart,zstart)];

%初始化
find_path=0;
gf=2;%增益因子

%当openlist列表不为空时
while isempty(open)==0 && find_path==0
%查找openlist中fx最小的节点
%返回openlist中每一个节点的f值
%返回fxlist（openlist中每一个节点的f值）中最小值的索引
%在openlist中取出f值最小的节点，赋值给cnode
   
     [xcurrent, ycurrent, zcurrent]=Binary_heap_pop(open);
     current_node=[xcurrent, ycurrent, zcurrent];
     


     %定义视线评估点（定义为[当前节点的父节点  当前节点]）
     xb=[came_fromx(xcurrent,ycurrent,zcurrent) xcurrent];
     yb=[came_fromy(xcurrent,ycurrent,zcurrent) ycurrent];
     zb=[came_fromz(xcurrent,ycurrent,zcurrent) zcurrent];

     %当来的节点(父母)和邻居之间的视线检查
     sight=is_sight_3D(xb,yb,zb,mapsize,E3d_safe); 

     %如果没有视线
     if sight==0 

         %初始化最小g值（Inf为无穷大）
         g_min=Inf;

         %检查的邻居节点
         for i=-1:1
             for j=-1:1
                 for k=0 
                     %如果相邻节点在网格内
                     if xcurrent+i>0 && ycurrent+j>0 && zcurrent+k>0 && xcurrent+i<=x_size && ycurrent+j<=y_size && zcurrent+k<=z_size

                         %如果邻居节点在关闭列表中
                         if max(sum([xcurrent+i==close(:,1) ycurrent+j==close(:,2) zcurrent+k==close(:,3)],2))==3

                             %计算从开始到邻居节点+从邻居节点到当前节点的距离
                             g_test=G(xcurrent+i,ycurrent+j,zcurrent+k)+sqrt(i^2+j^2+k^2);                            

                             %如果这个距离是最小的，保存它并指定当前节点的邻居为当前节点的父节点
                             if g_test<g_min

                                 g_min=g_test;  
                                 G(xcurrent,ycurrent,zcurrent)=g_test;

                                 came_fromx(xcurrent,ycurrent,zcurrent)=xcurrent+i;
                                 came_fromy(xcurrent,ycurrent,zcurrent)=ycurrent+j;
                                 came_fromz(xcurrent,ycurrent,zcurrent)=zcurrent+k;
                             end
                         end
                     end
                 end
             end
         end
     end

     
     
     

     %检查是否到达目标节点
     if xcurrent==xend && ycurrent==yend && zcurrent==zend
           %到达目标节点，找到了最短路径，退出并生成路径
           find_path=1;
        
     end
           %将openlist中取出f值最小的节点cnode放入到“关闭列表closelist”中
           close(size(close,1)+1,:)=current_node;
           %将openlist中f值最小的节点从“开启列表openlist“中删除
           open=Binary_heap_remove(open);
           % 扩展当前fx最小的节点，并进入下一次循环搜索
           % 检查的邻居节点
           for i=-1:1
               for j=-1:1
                   for k=0
                         %如果相邻节点在网格内
                         if xcurrent+i>0 && ycurrent+j>0 && zcurrent+k>0 && xcurrent+i<=x_size && ycurrent+j<=y_size && zcurrent+k<=z_size


                             %如果邻居节点既不属于open也不属于close
                             %%如果 A 是矩阵，则 sum(A) 将返回包含每列总和的行向量。
                             %%a(:,1)为取a矩阵中第一列元素；a(1,:)为取a矩阵中第一行元素。
                             %%S = sum(A,dim) 沿维度 dim 返回总和。例如，如果 A 为矩阵，则 sum(A,2) 是包含每一行总和的列向量。
                             %%xcurrent+j==open(:,1)返回 （open总行数）*1 数组
                             %check_open:检查邻居节点是否在open列表中，返回3，说明在open列表中。返回小于3，说明不在open列表中。
                             %check_closed：检查邻居节点是否在close列表中，返回3，说明在close列表中。返回小于3，说明不在close列表中。
                             check_open=max(sum([xcurrent+i==open(:,1) ycurrent+j==open(:,2) zcurrent+k==open(:,3)],2));
                             check_close=max(sum([xcurrent+i==close(:,1) ycurrent+j==close(:,2) zcurrent+k==close(:,3)],2));
                             check_obstacle=max(sum([xcurrent+i==obstacle(:,1) ycurrent+j==obstacle(:,2) zcurrent+k==obstacle(:,3)],2));

                             %isempty为空时，返回1；isempty为非空时，返回0。
                             if isempty(check_open)==1
                                 check_open=0;
                             end

                             if isempty(check_close)==1
                                 check_close=0;
                             end

                             if isempty(check_obstacle)==1
                                 check_obstacle=0;
                             end

                             % 判断邻居节点node是否在closelist、blocklist、lowrisklist中
                             if check_close==3 || check_obstacle==3
                                 continue
                             else
                                 % 经if语句判断得出，节点node既不在closelist中，也不在blocklist中
                                 % 如果邻居节点node不在openlist中
                                 if check_open<3
%                                      %在open中添加邻居节点node
%                                      %size(open,1)返回open的行数
%                                      open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      %计算从开始到当前节点的距离+当前节点到邻居节点的距离
%                                      g_try=G(xcurrent,ycurrent,zcurrent)+sqrt(i^2+j^2+k^2);
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
%                                      %记录从哪个节点到达邻居,存放父节点
%                                      came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=xcurrent; 
%                                      came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=ycurrent;
%                                      came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;
                                     G(xcurrent+i,ycurrent+j,zcurrent+k)=Inf;
                                     came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=0; 
                                     came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=0;
                                     came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=0;

                                 end
                                 G_old=G(xcurrent+i,ycurrent+j,zcurrent+k);
                                 
                                 % 经语句判断得出，节点node不在closelist中，不在blocklist中，在openlist中

                                 % 计算从起点到当前节点的父节点+从当前节点的父节点到当前节点的邻居的距离
                                 g_try=G(came_fromx(xcurrent,ycurrent,zcurrent),came_fromy(xcurrent,ycurrent,zcurrent),came_fromz(xcurrent,ycurrent,zcurrent))+sqrt((came_fromx(xcurrent,ycurrent,zcurrent)-(xcurrent+i))^2+(came_fromy(xcurrent,ycurrent,zcurrent)-(ycurrent+j))^2+(came_fromz(xcurrent,ycurrent,zcurrent)-(zcurrent+k))^2);

                                 % 如果这个距离小于邻居的距离
                                 if g_try<G(xcurrent+i,ycurrent+j,zcurrent+k)

                                     %在正确道路上，保存信息

                                     %记录从哪个节点到达邻居,存放父节点
                                     came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=came_fromx(xcurrent,ycurrent,zcurrent); 
                                     came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=came_fromy(xcurrent,ycurrent,zcurrent);
                                     came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=came_fromz(xcurrent,ycurrent,zcurrent);

                                     %评估成本函数
                                     G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
                                     H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
                                     %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                     F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
%                                      diatance_start_current=sqrt((xcurrent-xstart)^2+(ycurrent-ystart)^2+(zcurrent-zstart)^2);
%                                      diatance_end_current=sqrt((xcurrent-xend)^2+(ycurrent-yend)^2+(zcurrent-zend)^2);
%                                      aa=diatance_start_current/diatance_end_current;
%                                      obstacle_number=size(obstacle,1);
%                                      x_diatance_start_end=xstart-xend;
%                                      y_diatance_start_end=ystart-yend;
%                                      z_diatance_start_end=zstart-zend;
%                                      bb=obstacle_number/(abs(x_diatance_start_end)*abs(y_diatance_start_end)*abs(z_diatance_start_end));
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=aa*G(xcurrent+i,ycurrent+j,zcurrent+k)+((pi/2)-atan(bb))*H;
                           
                                 end
                                 if  G_old>G(xcurrent+i,ycurrent+j,zcurrent+k)
                                      if check_open==3
                                          open=Binary_heap_remove(open);
                                      end
                                          open=Binary_heap_insert(open,xcurrent+i,ycurrent+j,zcurrent+k,F(xcurrent+i,ycurrent+j,zcurrent+k));
                                     

%                                      if check_open<3
%                                             open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      end
%                                      %评估成本函数
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
                                      
                                    
                                 end
                                
                             end
                      


                         end
                   end
               end
           end
end


             
             
%逆向重建路径，知道从哪个节点到达好邻居


%第一个元素是到达点
path_backwards=[xcurrent,ycurrent,zcurrent];

%初始化
i=2;

%当起点还没有到达
while xcurrent~=xstart || ycurrent~=ystart || zcurrent~=zstart
    
    path_backwards(i,:)=[came_fromx(xcurrent,ycurrent,zcurrent) came_fromy(xcurrent,ycurrent,zcurrent) came_fromz(xcurrent,ycurrent,zcurrent)];
    xcurrent=path_backwards(i,1);
    ycurrent=path_backwards(i,2);    
    zcurrent=path_backwards(i,3); 
    i=i+1;
    
end

%路径节点数量
route_node_number=size(path_backwards,1);


%反向路径序列
path=path_backwards(route_node_number+1-(1:route_node_number),:);


%重新分配初始的开始和结束点
path(1,:)=[xstart ystart zstart];
path(route_node_number,:)=[xend yend zend];
    

end
     
 

        

