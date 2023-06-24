%lazy theta算法函数
function [ path,route_node_number,path_backwards1,path_backwards2 ] = Bi_lazy_theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,E3d_safe)

%环境矩阵(地图)的大小（x y z轴）
x_size=mapsize(1);
y_size=mapsize(2);
z_size=mapsize(3);

%定义到达邻居的节点
%%设mapsize为5*5*5的矩阵，则zeros(mapsize)会生成5*5*5的全零矩阵
%正向
came_fromx1=zeros(mapsize);
came_fromy1=zeros(mapsize);
came_fromz1=zeros(mapsize);
came_fromx1(xstart,ystart,zstart)=xstart;
came_fromy1(xstart,ystart,zstart)=ystart;
came_fromz1(xstart,ystart,zstart)=zstart;
%反向
came_fromx2=zeros(mapsize);
came_fromy2=zeros(mapsize);
came_fromz2=zeros(mapsize);
came_fromx2(xend,yend,zend)=xend;
came_fromy2(xend,yend,zend)=yend;
came_fromz2(xend,yend,zend)=zend;

%创建openlist，存放节点位置信息
%open=[xstart,ystart,zstart];
%创建closelist，存放节点位置信息
%正向
close1=[];
%反向
close2=[];
%计算设置G和F值。G:起点到当前节点的距离。F:当前节点到终点的距离。
%Inf*ones生成与mapsize维数相同的一个矩阵,矩阵的每个值都是无穷大
G1=Inf*ones(mapsize);
G1(xstart,ystart,zstart)=0;
F1=Inf*ones(mapsize);
F1(xstart,ystart,zstart)=sqrt((xend-xstart)^2+(yend-ystart)^2+(zend-zstart)^2);

%反向
G2=Inf*ones(mapsize);
G2(xend,yend,zend)=0;
F2=Inf*ones(mapsize);
F2(xend,yend,zend)=sqrt((xstart-xend)^2+(ystart-yend)^2+(zstart-zend)^2);

% g(sstart)=0;
% parent(sstart)=sstart; 
% open.insert(start,g(sstart)+h(start));
%正向
open1=[xstart,ystart,zstart,F1(xstart,ystart,zstart)];
%反向
open2=[xend,yend,zend,F2(xend,yend,zend)];

%初始化
find_path=0;
gf=2;%增益因子

%当openlist列表不为空时
while isempty(open1)==0 && isempty(open2)==0 && find_path==0
%查找openlist中fx最小的节点
%返回openlist中每一个节点的f值
%返回fxlist（openlist中每一个节点的f值）中最小值的索引
%在openlist中取出f值最小的节点，赋值给cnode
   
     [xcurrent1, ycurrent1, zcurrent1]=Binary_heap_pop(open1);
     current_node1=[xcurrent1, ycurrent1, zcurrent1];

     %定义视线评估点（定义为[当前节点的父节点  当前节点]）
     xb1=[came_fromx1(xcurrent1,ycurrent1,zcurrent1) xcurrent1];
     yb1=[came_fromy1(xcurrent1,ycurrent1,zcurrent1) ycurrent1];
     zb1=[came_fromz1(xcurrent1,ycurrent1,zcurrent1) zcurrent1];

     %当来的节点(父母)和邻居之间的视线检查
     sight=is_sight_3D(xb1,yb1,zb1,mapsize,E3d_safe); 

     %如果没有视线
     if sight==0 

         %初始化最小g值（Inf为无穷大）
         g_min=Inf;

         %检查的邻居节点
         for i=-1:1
             for j=-1:1
                 for k=-1:1 
                     %如果相邻节点在网格内
                     if xcurrent1+i>0 && ycurrent1+j>0 && zcurrent1+k>0 && xcurrent1+i<=x_size && ycurrent1+j<=y_size && zcurrent1+k<=z_size

                         %如果邻居节点在关闭列表中
                         if max(sum([xcurrent1+i==close1(:,1) ycurrent1+j==close1(:,2) zcurrent1+k==close1(:,3)],2))==3

                             %计算从开始到邻居节点+从邻居节点到当前节点的距离
                             g_test=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)+sqrt(i^2+j^2+k^2);                            

                             %如果这个距离是最小的，保存它并指定当前节点的邻居为当前节点的父节点
                             if g_test<g_min

                                 g_min=g_test;  
                                 G1(xcurrent1,ycurrent1,zcurrent1)=g_test;

                                 came_fromx1(xcurrent1,ycurrent1,zcurrent1)=xcurrent1+i;
                                 came_fromy1(xcurrent1,ycurrent1,zcurrent1)=ycurrent1+j;
                                 came_fromz1(xcurrent1,ycurrent1,zcurrent1)=zcurrent1+k;
                             end
                         end
                     end
                 end
             end
         end
     end
   
   
   

     [xcurrent2, ycurrent2, zcurrent2]=Binary_heap_pop(open2);
     current_node2=[xcurrent2, ycurrent2, zcurrent2];

     %定义视线评估点（定义为[当前节点的父节点  当前节点]）
     xb2=[came_fromx2(xcurrent2,ycurrent2,zcurrent2) xcurrent2];
     yb2=[came_fromy2(xcurrent2,ycurrent2,zcurrent2) ycurrent2];
     zb2=[came_fromz2(xcurrent2,ycurrent2,zcurrent2) zcurrent2];

     %当来的节点(父母)和邻居之间的视线检查
     sight=is_sight_3D(xb2,yb2,zb2,mapsize,E3d_safe); 

     %如果没有视线
     if sight==0 

         %初始化最小g值（Inf为无穷大）
         g_min=Inf;

         %检查的邻居节点
         for i=-1:1
             for j=-1:1
                 for k=-1:1 
                     %如果相邻节点在网格内
                     if xcurrent2+i>0 && ycurrent2+j>0 && zcurrent2+k>0 && xcurrent2+i<=x_size && ycurrent2+j<=y_size && zcurrent2+k<=z_size

                         %如果邻居节点在关闭列表中
                         if max(sum([xcurrent2+i==close2(:,1) ycurrent2+j==close2(:,2) zcurrent2+k==close2(:,3)],2))==3

                             %计算从开始到邻居节点+从邻居节点到当前节点的距离
                             g_test=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)+sqrt(i^2+j^2+k^2);                            

                             %如果这个距离是最小的，保存它并指定当前节点的邻居为当前节点的父节点
                             if g_test<g_min

                                 g_min=g_test;  
                                 G2(xcurrent2,ycurrent2,zcurrent2)=g_test;

                                 came_fromx1(xcurrent2,ycurrent2,zcurrent2)=xcurrent2+i;
                                 came_fromy1(xcurrent2,ycurrent2,zcurrent2)=ycurrent2+j;
                                 came_fromz1(xcurrent2,ycurrent2,zcurrent2)=zcurrent2+k;
                             end
                         end
                     end
                 end
             end
         end
     end
   
     
     

     %检查是否到达目标节点
     if xcurrent1==xcurrent2 && ycurrent1==ycurrent2 && zcurrent1==zcurrent2
           %到达目标节点，找到了最短路径，退出并生成路径
           find_path=1;
        
     end
           %将openlist中取出f值最小的节点cnode放入到“关闭列表closelist”中
           close1(size(close1,1)+1,:)=current_node1;
           %将openlist中f值最小的节点从“开启列表openlist“中删除
           open1=Binary_heap_remove(open1);
           
           %将openlist中取出f值最小的节点cnode放入到“关闭列表closelist”中
           close2(size(close2,1)+1,:)=current_node2;
           %将openlist中f值最小的节点从“开启列表openlist“中删除
           open2=Binary_heap_remove(open2);
           
           % 扩展当前fx最小的节点，并进入下一次循环搜索
           % 检查的邻居节点
           for i=-1:1
               for j=-1:1
                   for k=-1:1
                         %如果相邻节点在网格内
                         if xcurrent1+i>0 && ycurrent1+j>0 && zcurrent1+k>0 && xcurrent1+i<=x_size && ycurrent1+j<=y_size && zcurrent1+k<=z_size


                             %如果邻居节点既不属于open也不属于close
                             %%如果 A 是矩阵，则 sum(A) 将返回包含每列总和的行向量。
                             %%a(:,1)为取a矩阵中第一列元素；a(1,:)为取a矩阵中第一行元素。
                             %%S = sum(A,dim) 沿维度 dim 返回总和。例如，如果 A 为矩阵，则 sum(A,2) 是包含每一行总和的列向量。
                             %%xcurrent+j==open(:,1)返回 （open总行数）*1 数组
                             %check_open:检查邻居节点是否在open列表中，返回3，说明在open列表中。返回小于3，说明不在open列表中。
                             %check_closed：检查邻居节点是否在close列表中，返回3，说明在close列表中。返回小于3，说明不在close列表中。
                             check_open1=max(sum([xcurrent1+i==open1(:,1) ycurrent1+j==open1(:,2) zcurrent1+k==open1(:,3)],2));
                             check_close1=max(sum([xcurrent1+i==close1(:,1) ycurrent1+j==close1(:,2) zcurrent1+k==close1(:,3)],2));
                             check_obstacle=max(sum([xcurrent1+i==obstacle(:,1) ycurrent1+j==obstacle(:,2) zcurrent1+k==obstacle(:,3)],2));

                             %isempty为空时，返回1；isempty为非空时，返回0。
                             if isempty(check_open1)==1
                                 check_open1=0;
                             end

                             if isempty(check_close1)==1
                                 check_close1=0;
                             end

                             if isempty(check_obstacle)==1
                                 check_obstacle=0;
                             end

                             % 判断邻居节点node是否在closelist、blocklist、lowrisklist中
                             if check_close1==3 || check_obstacle==3
                                 continue
                             else
                                 % 经if语句判断得出，节点node既不在closelist中，也不在blocklist中
                                 % 如果邻居节点node不在openlist中
                                 if check_open1<3
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
                                     G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=Inf;
                                     came_fromx1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0; 
                                     came_fromy1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0;
                                     came_fromz1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0;

                                 end
                                 G_old1=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k);
                                 
                                 % 经语句判断得出，节点node不在closelist中，不在blocklist中，在openlist中

                                 % 计算从起点到当前节点的父节点+从当前节点的父节点到当前节点的邻居的距离
                                 g_try1=G1(came_fromx1(xcurrent1,ycurrent1,zcurrent1),came_fromy1(xcurrent1,ycurrent1,zcurrent1),came_fromz1(xcurrent1,ycurrent1,zcurrent1))+sqrt((came_fromx1(xcurrent1,ycurrent1,zcurrent1)-(xcurrent1+i))^2+(came_fromy1(xcurrent1,ycurrent1,zcurrent1)-(ycurrent1+j))^2+(came_fromz1(xcurrent1,ycurrent1,zcurrent1)-(zcurrent1+k))^2);

                                 % 如果这个距离小于邻居的距离
                                 if g_try1<G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)

                                     %在正确道路上，保存信息

                                     %记录从哪个节点到达邻居,存放父节点
                                     came_fromx1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromx1(xcurrent1,ycurrent1,zcurrent1); 
                                     came_fromy1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromy1(xcurrent1,ycurrent1,zcurrent1);
                                     came_fromz1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromz1(xcurrent1,ycurrent1,zcurrent1);

                                     %评估成本函数
                                     G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=g_try1;
                                     H1=sqrt((xend-(xcurrent1+i))^2+(yend-(ycurrent1+j))^2+(zend-(zcurrent1+k))^2);
                                     %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                     F1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)+gf*H1;
                                 end
                                 if  G_old1>G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)
                                      if check_open1==3
                                          open1=Binary_heap_remove(open1);
                                      end
                                          open1=Binary_heap_insert(open1,xcurrent1+i,ycurrent1+j,zcurrent1+k,F1(xcurrent1+i,ycurrent1+j,zcurrent1+k));
                                     

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
           
           % 扩展当前fx最小的节点，并进入下一次循环搜索
           % 检查的邻居节点
           for i=-1:1
               for j=-1:1
                   for k=-1:1
                         %如果相邻节点在网格内
                         if xcurrent2+i>0 && ycurrent2+j>0 && zcurrent2+k>0 && xcurrent2+i<=x_size && ycurrent2+j<=y_size && zcurrent2+k<=z_size


                             %如果邻居节点既不属于open也不属于close
                             %%如果 A 是矩阵，则 sum(A) 将返回包含每列总和的行向量。
                             %%a(:,1)为取a矩阵中第一列元素；a(1,:)为取a矩阵中第一行元素。
                             %%S = sum(A,dim) 沿维度 dim 返回总和。例如，如果 A 为矩阵，则 sum(A,2) 是包含每一行总和的列向量。
                             %%xcurrent+j==open(:,1)返回 （open总行数）*1 数组
                             %check_open:检查邻居节点是否在open列表中，返回3，说明在open列表中。返回小于3，说明不在open列表中。
                             %check_closed：检查邻居节点是否在close列表中，返回3，说明在close列表中。返回小于3，说明不在close列表中。
                             check_open2=max(sum([xcurrent2+i==open2(:,1) ycurrent2+j==open2(:,2) zcurrent2+k==open2(:,3)],2));
                             check_close2=max(sum([xcurrent2+i==close2(:,1) ycurrent2+j==close2(:,2) zcurrent2+k==close2(:,3)],2));
                             check_obstacle=max(sum([xcurrent2+i==obstacle(:,1) ycurrent2+j==obstacle(:,2) zcurrent2+k==obstacle(:,3)],2));

                             %isempty为空时，返回1；isempty为非空时，返回0。
                             if isempty(check_open2)==1
                                 check_open2=0;
                             end

                             if isempty(check_close2)==1
                                 check_close2=0;
                             end

                             if isempty(check_obstacle)==1
                                 check_obstacle=0;
                             end

                             % 判断邻居节点node是否在closelist、blocklist、lowrisklist中
                             if check_close2==3 || check_obstacle==3
                                 continue
                             else
                                 % 经if语句判断得出，节点node既不在closelist中，也不在blocklist中
                                 % 如果邻居节点node不在openlist中
                                 if check_open2<3
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
                                     G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=Inf;
                                     came_fromx2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0; 
                                     came_fromy2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0;
                                     came_fromz2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0;

                                 end
                                 G_old=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k);
                                 
                                 % 经语句判断得出，节点node不在closelist中，不在blocklist中，在openlist中

                                 % 计算从起点到当前节点的父节点+从当前节点的父节点到当前节点的邻居的距离
                                 g_try=G2(came_fromx2(xcurrent2,ycurrent2,zcurrent2),came_fromy2(xcurrent2,ycurrent2,zcurrent2),came_fromz2(xcurrent2,ycurrent2,zcurrent2))+sqrt((came_fromx2(xcurrent2,ycurrent2,zcurrent2)-(xcurrent2+i))^2+(came_fromy2(xcurrent1,ycurrent1,zcurrent1)-(ycurrent1+j))^2+(came_fromz2(xcurrent1,ycurrent1,zcurrent1)-(zcurrent1+k))^2);

                                 % 如果这个距离小于邻居的距离
                                 if g_try<G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)

                                     %在正确道路上，保存信息

                                     %记录从哪个节点到达邻居,存放父节点
                                     came_fromx2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromx2(xcurrent2,ycurrent2,zcurrent2); 
                                     came_fromy2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromy2(xcurrent2,ycurrent2,zcurrent2);
                                     came_fromz2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromz2(xcurrent2,ycurrent2,zcurrent2);

                                     %评估成本函数
                                     G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=g_try;
                                     H2=sqrt((xend-(xcurrent2+i))^2+(yend-(ycurrent2+j))^2+(zend-(zcurrent2+k))^2);
                                     %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                     F2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)+gf*H2;
                                 end
                                 if  G_old>G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)
                                      if check_open2==3
                                          open2=Binary_heap_remove(open2);
                                      end
                                          open2=Binary_heap_insert(open2,xcurrent2+i,ycurrent2+j,zcurrent2+k,F2(xcurrent2+i,ycurrent2+j,zcurrent2+k));
                                     

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
path_backwards1=[xcurrent1,ycurrent1,zcurrent1];

%初始化
i=2;

%当起点还没有到达
while xcurrent1~=xstart || ycurrent1~=ystart || zcurrent1~=zstart
    
    path_backwards1(i,:)=[came_fromx1(xcurrent1,ycurrent1,zcurrent1) came_fromy1(xcurrent1,ycurrent1,zcurrent1) came_fromz1(xcurrent1,ycurrent1,zcurrent1)];
    xcurrent1=path_backwards1(i,1);
    ycurrent1=path_backwards1(i,2);    
    zcurrent1=path_backwards1(i,3); 
    i=i+1;
    
end

%路径节点数量
route_node_number1=size(path_backwards1,1);


%反向路径序列
% path=path_backwards(route_node_number+1-(1:route_node_number),:);

path_backwards2=[xcurrent2,ycurrent2,zcurrent2];

%初始化
i=2;

%当起点还没有到达
while xcurrent2~=xend || ycurrent2~=yend || zcurrent2~=zend
    
    path_backwards2(i,:)=[came_fromx2(xcurrent2,ycurrent2,zcurrent2) came_fromy2(xcurrent2,ycurrent2,zcurrent2) came_fromz2(xcurrent2,ycurrent2,zcurrent2)];
    xcurrent2=path_backwards2(i,1);
    ycurrent2=path_backwards2(i,2);    
    zcurrent2=path_backwards2(i,3); 
    i=i+1;
    
end

%路径节点数量
route_node_number2=size(path_backwards2,1);
route_node_number = route_node_number1 + route_node_number2 - 1;

for i=1:route_node_number1-1
         path_backwards(i,:)=path_backwards1(route_node_number1+1-i,:);
end

path=[path_backwards;path_backwards2];
% path=path_backwards1(route_node_number+1-(1:route_node_number),:);


%重新分配初始的开始和结束点
path(1,:)=[xstart ystart zstart];
path(route_node_number,:)=[xend yend zend];
    

end
     
 

        

