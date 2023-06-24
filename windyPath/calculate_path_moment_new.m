% 计算每一时刻走过的路程及节点(高度每间隔5km)
% 函数参数：时间，路径节点，对经过路径节点计数 ,加入新节点的路径 
% 函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
function [x_new,y_new,count,path_new] = calculate_path_moment_new(time,path2,count,route_node_number2)
    path_distance2=zeros(route_node_number2,1);
    for i=2:route_node_number2 
        path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
    end
    %单位换算:1 千米/时=0.2777778 米/秒
    %800/3.6:km/h=m/s  time：时间   0.001：m=km
    time_path = (800/3.6)*time*0.001;
    for j=1:route_node_number2-1
        if time_path<path_distance2(j+1)
            % 本次走过的路径长度t_26_path < 起点到下一点之间的距离L 
            % 算出从起点到本次走过路径长度t_26_path后的点坐标point，该点坐标point称为新的起点
            xita = atan((path2(j+1,2)-path2(j,2))/(path2(j+1,1)-path2(j,1)));%角度
            x_new = path2(j,1) + time_path * cos(xita);%新节点x坐标
            y_new = path2(j,2) + time_path * sin(xita);%新节点y坐标
            x_new=round(x_new);%四舍五入
            y_new=round(y_new);%四舍五入
            break;
        elseif time_path == path_distance2(j+1)
            % 本次走过的路径长度t_26_path = 起点到下一点之间的距离L
            count = count + 1;%已经过一个改航点，所以count+1
            x_new = path2(j+1,1);%新节点x坐标
            y_new = path2(j+1,2);%新节点y坐标
            x_new=round(x_new);%四舍五入
            y_new=round(y_new);%四舍五入
            break;
        else
            count=count+1;
            continue;
        end
    end
    
 
        
    % 将新节点加入到原来的路径中
    if count==1
        path_new=[path2(count,:);x_new,y_new;path2([count+1:route_node_number2],:)];
    else
        path_new=[path2([1:count],:);x_new,y_new;path2([count+1:route_node_number2],:)];
    end
end

