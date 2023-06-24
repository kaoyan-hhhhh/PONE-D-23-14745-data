% 计算每一时刻走过的路程及节点(高度每间隔5km)
% 函数参数：时间，路径节点，对经过路径节点计数 ,加入新节点的路径 
% 函数返回值：新节点x坐标，新节点y坐标，对经过路径节点计数，路径节点总数
function [x_new,y_new,count,path_new] = calculate_path_moment(time,path2,count,route_node_number2)

    %单位换算:1 千米/时=0.2777778 米/秒
    %800/3.6:km/h=m/s  time：时间   0.001：m=km
    time_path = (800/3.6)*time*0.001;
    L = sqrt((path2(count+1,1)-path2(count,1))^2+(path2(count+1,2)-path2(count,2))^2);%起点到下一点之间的距离L
    LL = sqrt((path2(count+2,1)-path2(count+1,1))^2+(path2(count+2,2)-path2(count+1,2))^2);%下一点到下下一点之间的距离L
    % 本次走过的路径长度t_26_path 与 起点到下一点之间的距离L 的比较
    if time_path < L
        % 本次走过的路径长度t_26_path < 起点到下一点之间的距离L 
        % 算出从起点到本次走过路径长度t_26_path后的点坐标point，该点坐标point称为新的起点
        xita = atan((path2(count+1,2)-path2(count,2))/(path2(count+1,1)-path2(count,1)));%角度
        x_new = path2(count,1) + time_path * cos(xita);%新节点x坐标
        y_new = path2(count,2) + time_path * sin(xita);%新节点y坐标
        x_new=round(x_new);%四舍五入
        y_new=round(y_new);%四舍五入
    elseif time_path == L
        % 本次走过的路径长度t_26_path = 起点到下一点之间的距离L 
        count = count + 1;%已经过一个改航点，所以count+1
        x_new = path2(count,1);%新节点x坐标
        y_new = path2(count,2);%新节点y坐标
        x_new=round(x_new);%四舍五入
        y_new=round(y_new);%四舍五入
    else
        % 本次走过的路径长度t_26_path > 起点到下一点之间的距离L 
        count = count + 1;%已经过一个改航点，所以count+1
        % 剩余路径长度t_26_path_remain=本次走过的路径长度t_26_path-起点到下一点之间的距离L
        time_path_remain = time_path - L;
        % 算出从下一点到本次走过剩余路径长度Lnew后的点坐标point，该点坐标point称为新的起点
        xita = atan((path2(count+1,2)-path2(count,2))/(path2(count+1,1)-path2(count,1)));%角度
        x_new = path2(count,1) + time_path_remain * cos(xita);%新节点x坐标
        y_new = path2(count,2) + time_path_remain * sin(xita);%新节点y坐标
        x_new=round(x_new);%四舍五入
        y_new=round(y_new);%四舍五入
        if time_path > L+LL
            count = count + 1;%已经过一个改航点，所以count+1
        end
    end
    % 将新节点加入到原来的路径中
    if count==1
        path_new=[path2(count,:);x_new,y_new;path2([count+1:route_node_number2],:)];
    else
        path_new=[path2([1:count],:);x_new,y_new;path2([count+1:route_node_number2],:)];
    end
end

