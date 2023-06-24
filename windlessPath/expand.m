%% 基于边的扩展

% 1.确定扩展距离dis。
% 2. 计算边的法线向量v 
% 3. 确定扩展方向，判断依据是边向量和其法线向量的行列式的正负号，记为sign(sina)。
% 4. 对边上的两个顶点分别加上sign(sina)*dis*v得到新边的两个顶线。
% 5. 计算所有边的新顶点，并且根据新的顶点对计算其所在直线。 
% 6. 计算所有直线的交点即得到新多边形的顶点。

% 这种方法得到的多边形每条边和原多边形的完全平行的。
% close all;
% clc;
function [newp]=expand(p,ii)
dis = 10;   % 间隔10km
% p=[xx(k_3,1), xx(k_3,2)]; 


p(ii,:)=[];   % 19显示的是最后一个二维平面扩展的k值减去1，因为重复包含第一个点
D=p;
p=createSimplyPoly(D);  %创建简单多边形
p=[p;p(1,:)];   % 多边形最后一行都加一个起始点
% plot(p(:,1),p(:,2));


    R = [];
    revR = [];
    for i=1:length(p)-1             %计算每个顶点两条边的向量
        R = [R;p(i+1,:)-p(i,:)];
        if i==1
            revR =[revR;p(end-1,:)-p(1,:)];
        else
            revR =[revR;p(i-1,:)-p(i,:)];
        end
    end

    direct = [];
    sina=[];
    for i=1:length(R)               %计算顶点两条边单位向量求和并确定扩展方向
        D = R(i,:)/norm(R(i,:))+ revR(i,:)/norm(revR(i,:));
        sina = [sina;det([R(i,:);revR(i,:)])];
        direct = [direct;D/norm(D)];
    end

    newp = p;
    newp(1:end-1,:) = p(1:end-1,:) + dis*sign(sina).*direct;
    newp(end,:)=newp(1,:);

% figure;
% plot(p(:,1),p(:,2),'r-')

% axis equal;
hold on;
% newp2 =[newp2;newp2(1,:)];


newp =[newp;newp(1,:)];

% plot(newp2(:,1),newp2(:,2),'b-');

% set(gca, 'XLim',[min(p(:,1))-20 max(p(:,1))+20]); % X轴的数据显示范围
% set(gca, 'YLim',[min(p(:,2))-15 max(xx(:,2))+15]);
% grid on;
end

% axis equal;
% set(gca, 'XLim',[150 500]);   % X轴的数据显示范围
% set(gca, 'YLim',[-60 60]);
% set(gca,'xtick',[150:50:500]);