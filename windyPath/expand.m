%% ���ڱߵ���չ

% 1.ȷ����չ����dis��
% 2. ����ߵķ�������v 
% 3. ȷ����չ�����ж������Ǳ��������䷨������������ʽ�������ţ���Ϊsign(sina)��
% 4. �Ա��ϵ���������ֱ����sign(sina)*dis*v�õ��±ߵ��������ߡ�
% 5. �������бߵ��¶��㣬���Ҹ����µĶ���Լ���������ֱ�ߡ� 
% 6. ��������ֱ�ߵĽ��㼴�õ��¶���εĶ��㡣

% ���ַ����õ��Ķ����ÿ���ߺ�ԭ����ε���ȫƽ�еġ�
% close all;
% clc;
function [newp]=expand(p,ii)
dis = 10;   % ���10km
% p=[xx(k_3,1), xx(k_3,2)]; 


p(ii,:)=[];   % 19��ʾ�������һ����άƽ����չ��kֵ��ȥ1����Ϊ�ظ�������һ����
D=p;
p=createSimplyPoly(D);  %�����򵥶����
p=[p;p(1,:)];   % ��������һ�ж���һ����ʼ��
% plot(p(:,1),p(:,2));


    R = [];
    revR = [];
    for i=1:length(p)-1             %����ÿ�����������ߵ�����
        R = [R;p(i+1,:)-p(i,:)];
        if i==1
            revR =[revR;p(end-1,:)-p(1,:)];
        else
            revR =[revR;p(i-1,:)-p(i,:)];
        end
    end

    direct = [];
    sina=[];
    for i=1:length(R)               %���㶥�������ߵ�λ������Ͳ�ȷ����չ����
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

% set(gca, 'XLim',[min(p(:,1))-20 max(p(:,1))+20]); % X���������ʾ��Χ
% set(gca, 'YLim',[min(p(:,2))-15 max(xx(:,2))+15]);
% grid on;
end

% axis equal;
% set(gca, 'XLim',[150 500]);   % X���������ʾ��Χ
% set(gca, 'YLim',[-60 60]);
% set(gca,'xtick',[150:50:500]);