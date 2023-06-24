 %Main

clc
clear
%%
%%350秒时
%地图大小[y,x,z]
% mapsize=[500 120 80];
% mapsize=[100 100 100];
mapsize=[500 120];

%起点
%（10，-10）
xstart=10;
ystart=50;
% zstart=26;

% node_start=[xstart,ystart,zstart];
node_start=[xstart,ystart];

%目标点
%（490，-10）
xend=490;
yend=50;
% yend=80;
% zend=26;

%初始路径
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
% initial_path=[xstart,ystart-60;250,-5;xend,yend-60];

%发射前的路径规划（关闭大空域）
[before_fly_path,route_node_number,pathLengthMin,pathTime] = beforeFlyPath(xstart,ystart,xend,yend);

% node_end=[xend,yend,zend];
node_end=[xend,yend];

% path_pass矩阵存储已走过的节点
path_pass1=[xstart,ystart];
path_pass2=[xstart,ystart];
path_pass3=[xstart,ystart];

%obstacle=[5,15,12];
%obstacle=[];
%obstacle_number=20;
%获取随机障碍物
%[obstacle,map]=GetObstacle(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle_number,obstacle);
% [obstacle,ellipse_radius,ellipse_midpoint_x,ellipse_midpoint_y]=GetObstacle();
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_350_h_24();
% [obstacle2,ellipse_radius2,ellipse_midpoint_x2,ellipse_midpoint_y2]=GetObstacle();
% [obstacle3,ellipse_radius3,ellipse_midpoint_x3,ellipse_midpoint_y3]=GetObstacle();
% [obstacle4,ellipse_radius4,ellipse_midpoint_x4,ellipse_midpoint_y4]=GetObstacle();
obstacle_map=obstacle_map_zone(mapsize,obstacle1);


%测量路径计算时间
%tic
%生成路径(选择一个)
%[ path,route_node_number,open,close ] = a_star(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle);
%[ path,route_node_number,open,close ] = a_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle);
%[ path,route_node_number,open,close ] = a_star_heap1(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle);
%[ path,route_node_number,open,close ] = a_star_queue(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle);

%[ path,route_node_number,open,close ] = theta_star(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);

%[ path,route_node_number,open,close ] = theta_star2(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);
%[ path,route_node_number,open,close ] = theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);

%[ path,route_node_number,open,close ] = lazy_theta_star(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);
%[ path,route_node_number,open,close ] = lazy_theta_star2(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);
%[ path,route_node_number,open,close  ] = lazy_theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);
%[ path,route_node_number,open,close  ] = lazy_theta_star_queue(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);
%[ path,route_node_number,open,close  ] = Bi_lazy_theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,obstacle_map);

%% 2D
[ path1,route_node_number1,open1,close1 ] = a_star(xstart,ystart,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart,ystart,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart,ystart,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

%结束时间
%time_path=toc;



%%
% %计算路径长度(A*)
% path_distance=zeros(route_node_number,1);
% 
% % for i=2:route_node_number 
% % 	path_distance(i)=path_distance(i-1)+sqrt((path(i,1)-path(i-1,1))^2+(path(i,2)-path(i-1,2))^2+(path(i,3)-path(i-1,3))^2);      
% % end
% for i=2:route_node_number 
% 	path_distance(i)=path_distance(i-1)+sqrt((path(i,1)-path(i-1,1))^2+(path(i,2)-path(i-1,2))^2);      
% end
% 
% path_length=path_distance(route_node_number);


%%
% 计算每一时刻走过的路程(每隔12秒)
count=1;% 对经过路径节点计数
time=350-350;
% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);

% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];
    
%%
%Cov_trajectory_point();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
hold on;
%plot(path(:,1),path(:,2)-60,'g-','LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
%plot(path2(:,1),path2(:,2)-60,'r-','LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*）
% plot(path3(:,1),path3(:,2)-60,'y-','LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（theta*）

plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end

plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线
% hold on;
% plot(path(:,1),path(:,2)-60,'m.');%在规划好的路径上，标出经过的节点位置（红色 实心圆）
% legend('1','2','3','4');


axis tight
%将样式指定为 equal 以便沿着每个坐标轴使用相等的数据单位长度。
%axis equal
% axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5,0,mapsize(3)+2.5]);%设置坐标轴
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=350s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);

% for i=2:route_node_number 
% 	path_distance(i)=path_distance(i-1)+sqrt((path(i,1)-path(i-1,1))^2+(path(i,2)-path(i-1,2))^2+(path(i,3)-path(i-1,3))^2);      
% end
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_total_t_350_h_24=path_distance2(route_node_number2);
time2_t_350_h_24=path_length2_total_t_350_h_24/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);

% for i=2:route_node_number 
% 	path_distance(i)=path_distance(i-1)+sqrt((path(i,1)-path(i-1,1))^2+(path(i,2)-path(i-1,2))^2+(path(i,3)-path(i-1,3))^2);      
% end
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end

path_length3_total_t_350_h_24=path_distance3(route_node_number3);
time3_t_350_h_24=path_length3_total_t_350_h_24/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);

% for i=2:route_node_number 
% 	path_distance(i)=path_distance(i-1)+sqrt((path(i,1)-path(i-1,1))^2+(path(i,2)-path(i-1,2))^2+(path(i,3)-path(i-1,3))^2);      
% end
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end

path_length1_total_t_350_h_24=path_distance1(route_node_number1);
time1_t_350_h_24=path_length1_total_t_350_h_24/800;

%%
% t_410_h_21d5
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_410_h_21d5();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=410-350;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=410s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_410_h_21d5=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_410_h_21d5=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_410_h_21d5=path_length2_start_to_current_t_410_h_21d5+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_410_h_21d5=path_length2_current_to_end_t_410_h_21d5+path_length2_start_to_current_t_410_h_21d5;
time2_t_410_h_21d5=path_length2_total_t_410_h_21d5/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_410_h_21d5=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_410_h_21d5=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_410_h_21d5=path_length3_start_to_current_t_410_h_21d5+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_410_h_21d5=path_length3_current_to_end_t_410_h_21d5+path_length3_start_to_current_t_410_h_21d5;
time3_t_410_h_21d5=path_length3_total_t_410_h_21d5/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_410_h_21d5=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_410_h_21d5=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_410_h_21d5=path_length1_start_to_current_t_410_h_21d5+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_410_h_21d5=path_length1_current_to_end_t_410_h_21d5+path_length1_start_to_current_t_410_h_21d5;
time1_t_410_h_21d5=path_length1_total_t_410_h_21d5/800;

%%
% t_470_h_19d5
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_470_h_19d5();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=470-410;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）


% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=470s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_470_h_19d5=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_470_h_19d5=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_470_h_19d5=path_length2_start_to_current_t_470_h_19d5+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_470_h_19d5=path_length2_current_to_end_t_470_h_19d5+path_length2_start_to_current_t_470_h_19d5;
time2_t_470_h_19d5=path_length2_total_t_470_h_19d5/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_470_h_19d5=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_470_h_19d5=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_470_h_19d5=path_length3_start_to_current_t_470_h_19d5+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_470_h_19d5=path_length3_current_to_end_t_470_h_19d5+path_length3_start_to_current_t_470_h_19d5;
time3_t_470_h_19d5=path_length3_total_t_470_h_19d5/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_470_h_19d5=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_470_h_19d5=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_470_h_19d5=path_length1_start_to_current_t_470_h_19d5+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_470_h_19d5=path_length1_current_to_end_t_470_h_19d5+path_length1_start_to_current_t_470_h_19d5;
time1_t_470_h_19d5=path_length1_total_t_470_h_19d5/800;
%%
% t_530_h_17d7
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_530_h_17d7();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=530-470;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=530s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_530_h_17d7=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_530_h_17d7=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_530_h_17d7=path_length2_start_to_current_t_530_h_17d7+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_530_h_17d7=path_length2_current_to_end_t_530_h_17d7+path_length2_start_to_current_t_530_h_17d7;
time2_t_530_h_17d7=path_length2_total_t_530_h_17d7/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_530_h_17d7=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_530_h_17d7=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_530_h_17d7=path_length3_start_to_current_t_530_h_17d7+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_530_h_17d7=path_length3_current_to_end_t_530_h_17d7+path_length3_start_to_current_t_530_h_17d7;
time3_t_530_h_17d7=path_length3_total_t_530_h_17d7/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_530_h_17d7=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_530_h_17d7=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_530_h_17d7=path_length1_start_to_current_t_530_h_17d7+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_530_h_17d7=path_length1_current_to_end_t_530_h_17d7+path_length1_start_to_current_t_530_h_17d7;
time1_t_530_h_17d7=path_length1_total_t_530_h_17d7/800;
%%
% t_590_h_16d1
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_590_h_16d1();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);


% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=590-530;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）
plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=590s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_590_h_16d1=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_590_h_16d1=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_590_h_16d1=path_length2_start_to_current_t_590_h_16d1+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_590_h_16d1=path_length2_current_to_end_t_590_h_16d1+path_length2_start_to_current_t_590_h_16d1;
time2_t_590_h_16d1=path_length2_total_t_590_h_16d1/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_590_h_16d1=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_590_h_16d1=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_590_h_16d1=path_length3_start_to_current_t_590_h_16d1+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_590_h_16d1=path_length3_current_to_end_t_590_h_16d1+path_length3_start_to_current_t_590_h_16d1;
time3_t_590_h_16d1=path_length3_total_t_590_h_16d1/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_590_h_16d1=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_590_h_16d1=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_590_h_16d1=path_length1_start_to_current_t_590_h_16d1+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_590_h_16d1=path_length1_current_to_end_t_590_h_16d1+path_length1_start_to_current_t_590_h_16d1;
time1_t_590_h_16d1=path_length1_total_t_590_h_16d1/800;

%%
% t_650_h_14d7
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_650_h_14d7();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=650-590;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=650s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_650_h_14d7=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_650_h_14d7=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_650_h_14d7=path_length2_start_to_current_t_650_h_14d7+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_650_h_14d7=path_length2_current_to_end_t_650_h_14d7+path_length2_start_to_current_t_650_h_14d7;
time2_t_650_h_14d7=path_length2_total_t_650_h_14d7/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_650_h_14d7=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_650_h_14d7=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_650_h_14d7=path_length3_start_to_current_t_650_h_14d7+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_650_h_14d7=path_length3_current_to_end_t_650_h_14d7+path_length3_start_to_current_t_650_h_14d7;
time3_t_650_h_14d7=path_length3_total_t_650_h_14d7/800;


%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_650_h_14d7=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_650_h_14d7=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_650_h_14d7=path_length1_start_to_current_t_650_h_14d7+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_650_h_14d7=path_length1_current_to_end_t_650_h_14d7+path_length1_start_to_current_t_650_h_14d7;
time1_t_650_h_14d7=path_length1_total_t_650_h_14d7/800;
%%
% t_710_h_13d4
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_710_h_13d4();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=710-650;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=710s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_710_h_13d4=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_710_h_13d4=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_710_h_13d4=path_length2_start_to_current_t_710_h_13d4+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_710_h_13d4=path_length2_current_to_end_t_710_h_13d4+path_length2_start_to_current_t_710_h_13d4;
time2_t_710_h_13d4=path_length2_total_t_710_h_13d4/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_710_h_13d4=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_710_h_13d4=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_710_h_13d4=path_length3_start_to_current_t_710_h_13d4+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_710_h_13d4=path_length3_current_to_end_t_710_h_13d4+path_length3_start_to_current_t_710_h_13d4;
time3_t_710_h_13d4=path_length3_total_t_710_h_13d4/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_710_h_13d4=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_710_h_13d4=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_710_h_13d4=path_length1_start_to_current_t_710_h_13d4+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_710_h_13d4=path_length1_current_to_end_t_710_h_13d4+path_length1_start_to_current_t_710_h_13d4;
time1_t_710_h_13d4=path_length1_total_t_710_h_13d4/800;

%%
% t_770_h_12d2
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_770_h_12d2();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=770-710;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=770s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_770_h_12d2=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_770_h_12d2=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_770_h_12d2=path_length2_start_to_current_t_770_h_12d2+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_770_h_12d2=path_length2_current_to_end_t_770_h_12d2+path_length2_start_to_current_t_770_h_12d2;
time2_t_770_h_12d2=path_length2_total_t_770_h_12d2/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_770_h_12d2=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_770_h_12d2=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_770_h_12d2=path_length3_start_to_current_t_770_h_12d2+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_770_h_12d2=path_length3_current_to_end_t_770_h_12d2+path_length3_start_to_current_t_770_h_12d2;
time3_t_770_h_12d2=path_length3_total_t_770_h_12d2/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_770_h_12d2=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_770_h_12d2=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_770_h_12d2=path_length1_start_to_current_t_770_h_12d2+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_770_h_12d2=path_length1_current_to_end_t_770_h_12d2+path_length1_start_to_current_t_770_h_12d2;
time1_t_770_h_12d2=path_length1_total_t_770_h_12d2/800;

%%
% t_830_h_11d2
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_830_h_11d2();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=830-770;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=830s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_830_h_11d2=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_830_h_11d2=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_830_h_11d2=path_length2_start_to_current_t_830_h_11d2+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_830_h_11d2=path_length2_current_to_end_t_830_h_11d2+path_length2_start_to_current_t_830_h_11d2;
time2_t_830_h_11d2=path_length2_total_t_830_h_11d2/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_830_h_11d2=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_830_h_11d2=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_830_h_11d2=path_length3_start_to_current_t_830_h_11d2+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_830_h_11d2=path_length3_current_to_end_t_830_h_11d2+path_length3_start_to_current_t_830_h_11d2;
time3_t_830_h_11d2=path_length3_total_t_830_h_11d2/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_830_h_11d2=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_830_h_11d2=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_830_h_11d2=path_length1_start_to_current_t_830_h_11d2+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_830_h_11d2=path_length1_current_to_end_t_830_h_11d2+path_length1_start_to_current_t_830_h_11d2;
time1_t_830_h_11d2=path_length1_total_t_830_h_11d2/800;

%%
% t_890_h_10d2
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_890_h_10d2();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
% xstart1=191;
% ystart1=55;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=890-830;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=890s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_890_h_10d2=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_890_h_10d2=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_890_h_10d2=path_length2_start_to_current_t_890_h_10d2+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_890_h_10d2=path_length2_current_to_end_t_890_h_10d2+path_length2_start_to_current_t_890_h_10d2;
time2_t_890_h_10d2=path_length2_total_t_890_h_10d2/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_890_h_10d2=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_890_h_10d2=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_890_h_10d2=path_length3_start_to_current_t_890_h_10d2+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_890_h_10d2=path_length3_current_to_end_t_890_h_10d2+path_length3_start_to_current_t_890_h_10d2;
time3_t_890_h_10d2=path_length3_total_t_890_h_10d2/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_890_h_10d2=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_890_h_10d2=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_890_h_10d2=path_length1_start_to_current_t_890_h_10d2+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_890_h_10d2=path_length1_current_to_end_t_890_h_10d2+path_length1_start_to_current_t_890_h_10d2;
time1_t_890_h_10d2=path_length1_total_t_890_h_10d2/800;
%%
% t_950_h_9d2
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_950_h_9d2();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=950-890;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=950s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_950_h_9d2=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_950_h_9d2=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_950_h_9d2=path_length2_start_to_current_t_950_h_9d2+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_950_h_9d2=path_length2_current_to_end_t_950_h_9d2+path_length2_start_to_current_t_950_h_9d2;
time2_t_950_h_9d2=path_length2_total_t_950_h_9d2/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_950_h_9d2=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_950_h_9d2=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_950_h_9d2=path_length3_start_to_current_t_950_h_9d2+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_950_h_9d2=path_length3_current_to_end_t_950_h_9d2+path_length3_start_to_current_t_950_h_9d2;
time3_t_950_h_9d2=path_length3_total_t_950_h_9d2/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_950_h_9d2=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_950_h_9d2=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_950_h_9d2=path_length1_start_to_current_t_950_h_9d2+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_950_h_9d2=path_length1_current_to_end_t_950_h_9d2+path_length1_start_to_current_t_950_h_9d2;
time1_t_950_h_9d2=path_length1_total_t_950_h_9d2/800;

%%
% t_1010_h_8d4
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1010_h_8d4();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=1010-950;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=1010s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1010_h_8d4=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1010_h_8d4=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1010_h_8d4=path_length2_start_to_current_t_1010_h_8d4+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1010_h_8d4=path_length2_current_to_end_t_1010_h_8d4+path_length2_start_to_current_t_1010_h_8d4;
time2_t_1010_h_8d4=path_length2_total_t_1010_h_8d4/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1010_h_8d4=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1010_h_8d4=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1010_h_8d4=path_length3_start_to_current_t_1010_h_8d4+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1010_h_8d4=path_length3_current_to_end_t_1010_h_8d4+path_length3_start_to_current_t_1010_h_8d4;
time3_t_1010_h_8d4=path_length3_total_t_1010_h_8d4/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1010_h_8d4=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1010_h_8d4=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1010_h_8d4=path_length1_start_to_current_t_1010_h_8d4+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1010_h_8d4=path_length1_current_to_end_t_1010_h_8d4+path_length1_start_to_current_t_1010_h_8d4;
time1_t_1010_h_8d4=path_length1_total_t_1010_h_8d4/800;

%%
% t_1070_h_7d6
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1070_h_7d6();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=1070-1010;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=1070s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1070_h_7d6=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1070_h_7d6=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1070_h_7d6=path_length2_start_to_current_t_1070_h_7d6+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1070_h_7d6=path_length2_current_to_end_t_1070_h_7d6+path_length2_start_to_current_t_1070_h_7d6;
time2_t_1070_h_7d6=path_length2_total_t_1070_h_7d6/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1070_h_7d6=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1070_h_7d6=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1070_h_7d6=path_length3_start_to_current_t_1070_h_7d6+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1070_h_7d6=path_length3_current_to_end_t_1070_h_7d6+path_length3_start_to_current_t_1070_h_7d6;
time3_t_1070_h_7d6=path_length3_total_t_1070_h_7d6/800;


%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1070_h_7d6=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1070_h_7d6=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1070_h_7d6=path_length1_start_to_current_t_1070_h_7d6+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1070_h_7d6=path_length1_current_to_end_t_1070_h_7d6+path_length1_start_to_current_t_1070_h_7d6;
time1_t_1070_h_7d6=path_length1_total_t_1070_h_7d6/800;
%%
% t_1130_h_6d8
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1130_h_6d8();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=1130-1070;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=1130s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1130_h_6d8=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1130_h_6d8=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1130_h_6d8=path_length2_start_to_current_t_1130_h_6d8+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1130_h_6d8=path_length2_current_to_end_t_1130_h_6d8+path_length2_start_to_current_t_1130_h_6d8;
time2_t_1130_h_6d8=path_length2_total_t_1130_h_6d8/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1130_h_6d8=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1130_h_6d8=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1130_h_6d8=path_length3_start_to_current_t_1130_h_6d8+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1130_h_6d8=path_length3_current_to_end_t_1130_h_6d8+path_length3_start_to_current_t_1130_h_6d8;
time3_t_1130_h_6d8=path_length3_total_t_1130_h_6d8/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1130_h_6d8=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1130_h_6d8=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1130_h_6d8=path_length1_start_to_current_t_1130_h_6d8+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1130_h_6d8=path_length1_current_to_end_t_1130_h_6d8+path_length1_start_to_current_t_1130_h_6d8;
time1_t_1130_h_6d8=path_length1_total_t_1130_h_6d8/800;
%%
% t_1190_h_6d1
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1190_h_6d1();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=1190-1130;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=1190s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1190_h_6d1=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1190_h_6d1=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1190_h_6d1=path_length2_start_to_current_t_1190_h_6d1+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1190_h_6d1=path_length2_current_to_end_t_1190_h_6d1+path_length2_start_to_current_t_1190_h_6d1;
time2_t_1190_h_6d1=path_length2_total_t_1190_h_6d1/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1190_h_6d1=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1190_h_6d1=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1190_h_6d1=path_length3_start_to_current_t_1190_h_6d1+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1190_h_6d1=path_length3_current_to_end_t_1190_h_6d1+path_length3_start_to_current_t_1190_h_6d1;
time3_t_1190_h_6d1=path_length3_total_t_1190_h_6d1/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1190_h_6d1=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1190_h_6d1=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1190_h_6d1=path_length1_start_to_current_t_1190_h_6d1+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1190_h_6d1=path_length1_current_to_end_t_1190_h_6d1+path_length1_start_to_current_t_1190_h_6d1;
time1_t_1190_h_6d1=path_length1_total_t_1190_h_6d1/800;
%%
% t_1250_h_5d4
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1250_h_5d4();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);

% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
time=1250-1190;
[x_new1,y_new1,count1,path_new1] = calculate_path_moment_new(time,path1,count,route_node_number1);
[x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
[x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
path_pass1=[path_pass1;x_new1,y_new1];
path_pass2=[path_pass2;x_new2,y_new2];
path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path_new1(:,1),path_new1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线


axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('t=1250s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1250_h_5d4=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1250_h_5d4=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1250_h_5d4=path_length2_start_to_current_t_1250_h_5d4+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1250_h_5d4=path_length2_current_to_end_t_1250_h_5d4+path_length2_start_to_current_t_1250_h_5d4;
time2_t_1250_h_5d4=path_length2_total_t_1250_h_5d4/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1250_h_5d4=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1250_h_5d4=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1250_h_5d4=path_length3_start_to_current_t_1250_h_5d4+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1250_h_5d4=path_length3_current_to_end_t_1250_h_5d4+path_length3_start_to_current_t_1250_h_5d4;
time3_t_1250_h_5d4=path_length3_total_t_1250_h_5d4/800;

%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1250_h_5d4=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1250_h_5d4=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1250_h_5d4=path_length1_start_to_current_t_1250_h_5d4+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1250_h_5d4=path_length1_current_to_end_t_1250_h_5d4+path_length1_start_to_current_t_1250_h_5d4;
time1_t_1250_h_5d4=path_length1_total_t_1250_h_5d4/800;

%%
% t_1250之后
mapsize=[500 120];
%（10，-10）
xstart=10;
ystart=50;
node_start=[xstart,ystart];
%（490，-10）
xend=490;
yend=50;
%（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
initial_path=[-230,-15;250,-5;730,-15];
node_end=[xend,yend];
[obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1]=GetObstacle_t_1250_after();
rectangle('Position',[130 -30 310 48],'EdgeColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);%画禁飞区（大空域；发射前划设）
hold on;
plot(xstart,ystart-60,'r>','LineWidth',1.5);% 画出起点(绿色 正方形)
hold on;
plot(xend,yend-60,'k>','LineWidth',1.5);%画出目标点（红色 空心圆）
obstacle_map=obstacle_map_zone(mapsize,obstacle1);
xstart1=x_new1;
ystart1=y_new1;
xstart2=x_new2;
ystart2=y_new2;
xstart3=x_new3;
ystart3=y_new3;
%当前节点到目标节点的路径规划
[ path2,route_node_number2,open2,close2 ] = lazy_theta_star2(xstart2,ystart2,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path3,route_node_number3,open3,close3 ] = lazy_theta_star2_fengxian(xstart3,ystart3,xend,yend,mapsize,obstacle1,obstacle_map,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);
[ path1,route_node_number1,open1,close1 ] = a_star(xstart1,ystart1,xend,yend,mapsize,obstacle1,ellipse_radius1,ellipse_midpoint_x1,ellipse_midpoint_y1);


% 函数参数：时间，路径节点，对经过路径节点计数  函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
% time=1310-1250;
% [x_new2,y_new2,count2,path_new2] = calculate_path_moment(time,path2,count,route_node_number2);
% [x_new3,y_new3,count3,path_new3] = calculate_path_moment(time,path3,count,route_node_number3);
% path_pass矩阵存储已走过的节点
% path_pass2=[path_pass2;x_new2,y_new2];
% path_pass3=[path_pass3;x_new3,y_new3];

hold on;
plot(before_fly_path(:,1),before_fly_path(:,2),'--','color',[0.4940 0.1840 0.5560],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（发射前禁飞区路径 Lazy theta*）
% plot(path_new3(:,1),path_new3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
% plot(path_new2(:,1),path_new2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path3(:,1),path3(:,2)-60,'-','color',[0.8500 0.3250 0.0980],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（基于风险的Lazy theta*）
plot(path2(:,1),path2(:,2)-60,'--','color',[0.4660 0.6740 0.1880],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*，完整路径）
plot(path1(:,1),path1(:,2)-60,'--','color',[0.6350 0.0780 0.1840],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

% if count==1
%     plot(path_new([count,count+1],1),path_new([count,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% else
%     plot(path_new([1,count+1],1),path_new([1,count+1],2)-60,'-','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
% end
plot(path_pass3(:,1),path_pass3(:,2)-60,'-','color',[0 0.4470 0.7410],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass2(:,1),path_pass2(:,2)-60,'--','color',[0.3010 0.7450 0.9330],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（Lazy theta*,已走过）
plot(path_pass1(:,1),path_pass1(:,2)-60,'--','color',[0.9290 0.6940 0.1250],'LineWidth',1.5);%绘制由线段连接的一组坐标，即规划好的路径（A*）

plot(initial_path(:,1),initial_path(:,2),'b--','LineWidth',1.5);%初始航线

axis tight
axis([60,mapsize(1)+2.5,35,mapsize(2)+2.5]);%设置坐标轴
% xlabel('x');%为 x 轴添加标签
% ylabel('y');%为 y 轴添加标签
title('After 1250s');%添加标题
grid off;

%计算路径长度(Lazy theta*)
path_distance2=zeros(route_node_number2,1);
for i=2:route_node_number2 
	path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length2_current_to_end_t_1250_after=path_distance2(route_node_number2);
%起点到当前节点的路径长度
hangshu2=size(path_pass2,1);
path_length2_start_to_current_t_1250_after=0;
for i=1:hangshu2-1
    path_length2_start_to_current_t_1250_after=path_length2_start_to_current_t_1250_after+sqrt((path_pass2(i+1,1)-path_pass2(i,1))^2+(path_pass2(i+1,2)-path_pass2(i,2))^2);
end
%起点到目标节点的路径长度
path_length2_total_t_1250_after=path_length2_current_to_end_t_1250_after+path_length2_start_to_current_t_1250_after;
time2_t_1250_after=path_length2_total_t_1250_after/800;

%计算路径长度(基于风险的Lazy theta*)
path_distance3=zeros(route_node_number3,1);
for i=2:route_node_number3 
	path_distance3(i)=path_distance3(i-1)+sqrt((path3(i,1)-path3(i-1,1))^2+(path3(i,2)-path3(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length3_current_to_end_t_1250_after=path_distance3(route_node_number3);
%起点到当前节点的路径长度
hangshu3=size(path_pass3,1);
path_length3_start_to_current_t_1250_after=0;
for i=1:hangshu3-1
    path_length3_start_to_current_t_1250_after=path_length3_start_to_current_t_1250_after+sqrt((path_pass3(i+1,1)-path_pass3(i,1))^2+(path_pass3(i+1,2)-path_pass3(i,2))^2);
end
%起点到目标节点的路径长度
path_length3_total_t_1250_after=path_length3_current_to_end_t_1250_after+path_length3_start_to_current_t_1250_after;
time3_t_1250_after=path_length3_total_t_1250_after/800;


%计算路径长度(A*)
path_distance1=zeros(route_node_number1,1);
for i=2:route_node_number1 
	path_distance1(i)=path_distance1(i-1)+sqrt((path1(i,1)-path1(i-1,1))^2+(path1(i,2)-path1(i-1,2))^2);      
end
%当前节点(起点)到目标节点的路径长度
path_length1_current_to_end_t_1250_after=path_distance1(route_node_number1);
%起点到当前节点的路径长度
hangshu1=size(path_pass1,1);
path_length1_start_to_current_t_1250_after=0;
for i=1:hangshu1-1
    path_length1_start_to_current_t_1250_after=path_length1_start_to_current_t_1250_after+sqrt((path_pass1(i+1,1)-path_pass1(i,1))^2+(path_pass1(i+1,2)-path_pass1(i,2))^2);
end
%起点到目标节点的路径长度
path_length1_total_t_1250_after=path_length1_current_to_end_t_1250_after+path_length1_start_to_current_t_1250_after;
time1_t_1250_after=path_length1_total_t_1250_after/800;