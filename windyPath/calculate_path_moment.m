% ����ÿһʱ���߹���·�̼��ڵ�(�߶�ÿ���5km)
% ����������ʱ�䣬·���ڵ㣬�Ծ���·���ڵ���� ,�����½ڵ��·�� 
% ��������ֵ���½ڵ�x���꣬�½ڵ�y���꣬�Ծ���·���ڵ������·���ڵ�����
function [x_new,y_new,count,path_new] = calculate_path_moment(time,path2,count,route_node_number2)

    %��λ����:1 ǧ��/ʱ=0.2777778 ��/��
    %800/3.6:km/h=m/s  time��ʱ��   0.001��m=km
    time_path = (800/3.6)*time*0.001;
    L = sqrt((path2(count+1,1)-path2(count,1))^2+(path2(count+1,2)-path2(count,2))^2);%��㵽��һ��֮��ľ���L
    LL = sqrt((path2(count+2,1)-path2(count+1,1))^2+(path2(count+2,2)-path2(count+1,2))^2);%��һ�㵽����һ��֮��ľ���L
    % �����߹���·������t_26_path �� ��㵽��һ��֮��ľ���L �ıȽ�
    if time_path < L
        % �����߹���·������t_26_path < ��㵽��һ��֮��ľ���L 
        % �������㵽�����߹�·������t_26_path��ĵ�����point���õ�����point��Ϊ�µ����
        xita = atan((path2(count+1,2)-path2(count,2))/(path2(count+1,1)-path2(count,1)));%�Ƕ�
        x_new = path2(count,1) + time_path * cos(xita);%�½ڵ�x����
        y_new = path2(count,2) + time_path * sin(xita);%�½ڵ�y����
        x_new=round(x_new);%��������
        y_new=round(y_new);%��������
    elseif time_path == L
        % �����߹���·������t_26_path = ��㵽��һ��֮��ľ���L 
        count = count + 1;%�Ѿ���һ���ĺ��㣬����count+1
        x_new = path2(count,1);%�½ڵ�x����
        y_new = path2(count,2);%�½ڵ�y����
        x_new=round(x_new);%��������
        y_new=round(y_new);%��������
    else
        % �����߹���·������t_26_path > ��㵽��һ��֮��ľ���L 
        count = count + 1;%�Ѿ���һ���ĺ��㣬����count+1
        % ʣ��·������t_26_path_remain=�����߹���·������t_26_path-��㵽��һ��֮��ľ���L
        time_path_remain = time_path - L;
        % �������һ�㵽�����߹�ʣ��·������Lnew��ĵ�����point���õ�����point��Ϊ�µ����
        xita = atan((path2(count+1,2)-path2(count,2))/(path2(count+1,1)-path2(count,1)));%�Ƕ�
        x_new = path2(count,1) + time_path_remain * cos(xita);%�½ڵ�x����
        y_new = path2(count,2) + time_path_remain * sin(xita);%�½ڵ�y����
        x_new=round(x_new);%��������
        y_new=round(y_new);%��������
        if time_path > L+LL
            count = count + 1;%�Ѿ���һ���ĺ��㣬����count+1
        end
    end
    % ���½ڵ���뵽ԭ����·����
    if count==1
        path_new=[path2(count,:);x_new,y_new;path2([count+1:route_node_number2],:)];
    else
        path_new=[path2([1:count],:);x_new,y_new;path2([count+1:route_node_number2],:)];
    end
end

