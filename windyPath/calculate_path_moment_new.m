% ����ÿһʱ���߹���·�̼��ڵ�(�߶�ÿ���5km)
% ����������ʱ�䣬·���ڵ㣬�Ծ���·���ڵ���� ,�����½ڵ��·�� 
% ��������ֵ���½ڵ�x���꣬�½ڵ�y���꣬�Ծ���·���ڵ������·���ڵ�����
function [x_new,y_new,count,path_new] = calculate_path_moment_new(time,path2,count,route_node_number2)
    path_distance2=zeros(route_node_number2,1);
    for i=2:route_node_number2 
        path_distance2(i)=path_distance2(i-1)+sqrt((path2(i,1)-path2(i-1,1))^2+(path2(i,2)-path2(i-1,2))^2);      
    end
    %��λ����:1 ǧ��/ʱ=0.2777778 ��/��
    %800/3.6:km/h=m/s  time��ʱ��   0.001��m=km
    time_path = (800/3.6)*time*0.001;
    for j=1:route_node_number2-1
        if time_path<path_distance2(j+1)
            % �����߹���·������t_26_path < ��㵽��һ��֮��ľ���L 
            % �������㵽�����߹�·������t_26_path��ĵ�����point���õ�����point��Ϊ�µ����
            xita = atan((path2(j+1,2)-path2(j,2))/(path2(j+1,1)-path2(j,1)));%�Ƕ�
            x_new = path2(j,1) + time_path * cos(xita);%�½ڵ�x����
            y_new = path2(j,2) + time_path * sin(xita);%�½ڵ�y����
            x_new=round(x_new);%��������
            y_new=round(y_new);%��������
            break;
        elseif time_path == path_distance2(j+1)
            % �����߹���·������t_26_path = ��㵽��һ��֮��ľ���L
            count = count + 1;%�Ѿ���һ���ĺ��㣬����count+1
            x_new = path2(j+1,1);%�½ڵ�x����
            y_new = path2(j+1,2);%�½ڵ�y����
            x_new=round(x_new);%��������
            y_new=round(y_new);%��������
            break;
        else
            count=count+1;
            continue;
        end
    end
    
 
        
    % ���½ڵ���뵽ԭ����·����
    if count==1
        path_new=[path2(count,:);x_new,y_new;path2([count+1:route_node_number2],:)];
    else
        path_new=[path2([1:count],:);x_new,y_new;path2([count+1:route_node_number2],:)];
    end
end

