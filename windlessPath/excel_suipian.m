clc;
close all;
clear;
%��ʼֵ�趨
%��ʼֵ�趨
% ��������
Point=10;
col=Point;
row=Point;
NN= col*row;
O = zeros(3);
I = eye(3);
B=[O;I];
vt0 = zeros(3,NN);

%��ֵ
z_0 = 75*1000;                            %(m)��ʼ�߶�
psi0 = 0;                                %(��)heading angle
gamma0 = convang(-1,'deg','rad');        %(��)flight path angle
v0=7.2*1000;                             %(m/s) relative speed
theta0 = convang(102,'deg','rad');      %(��W)����
phi0 = convang(28,'deg','rad');          %(��N)γ��
beta = 5;                                %(kg/m2)����ϵ��������һ������ϵ�������о���
%�������
omegae = 72.9217*10^(-6);                %��ת���ٶ�
re = 6372.8*1000;                        %(m)����뾶
ge = 9.81;                               %(m/s^2)�����������ٶ�
Delta_v = 100;                           %��v(m/s)

% �洢��Ƭλ�ã�ʱ��������λ����Ϣ��
suipian1=[];
suipian=[];
%% �ڹ�ʽ43������ȡ��͵ȼ��ȡ�����������ķֲ����û�仯 
%��ʼ���ٶȷ������ȡ��
% phi =pi*rand([1,col])-(pi/2);   %����ֲ�դ��,rand([1,col])��ʾ1�У�col����0��1������ȡ�����
% theta =2*pi*rand([1,row]);

% ��ʼ���ٶȷ��򣬵ȼ��ȡ��
phi = linspace(-pi/2,pi/2,col);
theta = linspace(0,2*pi,col);             % 0-180�Ƕ�����180-360��Ӧ��������180-0
%%
V0 = diag([0.0025,0.0025,0.0053]);       %���ٶ�ʸ��Э�������

N=3000;                                 %����������
%��ʼ��
x1=zeros(1,N);
x2=zeros(1,N);
x3=zeros(1,N);
v1=zeros(1,N);
v2=zeros(1,N);
v3=zeros(1,N);

%ƽ�����ٶȷ���
x1(1)=0;    % ��ըǰһʱ�̵�λ��
x2(1)=0;
x3(1)=z_0;

v1(1)=v0*cos(gamma0)*cos(psi0);   % ��ըǰһʱ�̵��ٶ�
v2(1)=v0*cos(gamma0)*sin(psi0);
v3(1)=v0*sin(gamma0);

%��ʼλ��ʸ�����ٶ�ʸ��
x0e=[x1(1),x2(1),x3(1)]';
v0e = [v1(1),v2(1),v3(1)]';

ez=zeros(6*N,NN);    % N��������
Ez=zeros(6*N,1);    % N��������
Z=zeros(6*N,6);
z_stand = zeros(6*N,NN); %�洢����˶�״̬
%% ������ƹ켣
% �Էɻ�����ǰ��״̬��3��λ�÷�����3���ٶȷ������������˶�����10-15�������ƹ켣��
for n=1:N-1
    % w=[0,0,0]'; %�����Ϊ0
    z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
    w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0ʱ�̣�ʵʩˮƽ��ģ��
    
    % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%���������ܶ�ģ��MSISE-00�������Ƿ����ɢ���״���չ2000����T�¶ȣ�rho�ܶ�
    rhoe = 1.752;                         %(kg/m^3)
    H = 6.7*1000;                              %(km)
    rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);
    h=1;
    g = ge*(re/(re+x3(n)))^2;     % �淽����ģ��
    vn=[v1(n),v2(n),v3(n)]';      %
    va=vn-w;
    va_norm=norm(va); %��������ʵ�ģ
    %��ά�˶�����
    [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
        stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %���Ĺ�ʽ
    end_num=n;
    if (x3(n+1)<0)
        end_num=n-1;
        break;
    end
    
end
z_stand = repmat(z_stand(:,1),[1,NN]);

%% ��ƹ켣��ͼ
v1_r=v1(1:end_num);
v2_r=v2(1:end_num);
v3_r=v3(1:end_num);
x1_r=x1(1:end_num)/1000;  %����ͼ�����ᵥλΪkm
x2_r=x2(1:end_num)/1000;
x3_r=x3(1:end_num)/1000;

%��ƹ켣���꣬���Ƿ���ͼ�еĹ켣
%ͨ���Ƶ���άƽ���˶����̣�������ǰ��״̬����Ϊ��ʼ״̬����ƽ������ϵ��Ϊ��׼���õ���һ����Ƶ�����
x_nt=[x1_r;x2_r;x3_r];       %�����У�26��   ��ƹ켣���ʽ,3��1799
v_nt=[v1_r;v2_r;v3_r];

% plot3(x1_r,x2_r,x3_r,'LineWidth',0.8);       % ��3D�ռ��л��ƻ�����ƹ켣x_nt
view([55 32]);               % ����λ������Ϊ55�ȣ����߳�����Ϊ32��
% set(gca,'YDir','reverse');   % ��Y�ᷴ����
set(gca, 'XLim',[0 500]);   % X���������ʾ��Χ
set(gca, 'YLim',[-60 60]);
set(gca, 'ZLim',[0 20]);

xlabel('x/km');
ylabel('y/km');
zlabel('z/km');
grid on;
% set(gca, 'GridLineStyle', ':');  % ����Ϊ����
% title('18*18��Э�����㷨(�з�)');

%% ����������Ƭ��ʼ�ٶ�(NN���������㣩�켣���ٶ�)
for i = 1:col
    for j = 1:row
        
        vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
        sn0 = [x0e',v0e']';  % s*0��ƹ켣
        s0 = [x0e',vt0(:,(i-1)*col+j)']';  % ʵ��״̬����
        
        z0 = s0-sn0;    % ʵ�� - ���� = ƫ��о�ƫ�ʵ�ʲ�ȷ���ԣ�������������̣�ͨ��Э�������о������ķ�����
        Z0 = [O,O;O,V0];     % Э�������
        ez(1:6,(i-1)*col+j) = z0;   % NN�����������ֵ�ĳ�ʼ����    !!!!!
        Z(1:6,1:6)=Z0;    % �洢Э�������
    end
end

%% ����Ƭ���˶�ɢ��
for tn = 1:end_num
    %         h = 1;
    if tn > 1 %�þ���γ�������߶ȱ仯�ı仯�ٶ���Ϊƫ��
        w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
        w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
        hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
        hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
        hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
        dw = (w2-w1)./2/hw'; %d��
        ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
    else
        dw = [0.0026 0.0015 0];
        ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
    end
    
    g = ge*(re/(re+x3(n)))^2;     %�淽����ģ��
    vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %��ǰ���v(123)
    va = vn - w;
    va_norm = norm(va);  %��Կ����ٶȵĴ�С
    
    rhoe = 1.752;   %(kg/m^3)
    H = 6.7*1000;   %(km)
    rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %�����ܶ�@�ú��θ߶ȣ�x3��
    drho1 = 0;
    drho2 = 0;     %p��x2��ƫ��
    drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %��ʽ��17��,����p��x3��ƫ��
    drho = [drho1,drho2 drho3]';
    
    % ʱ��ϵ������A(t);
    F = zeros(3);   %3*3������󣬳�ʼ��
    G = zeros(3);
    delta = eye(3);
    
    for i = 1:3
        for j = 1:3
            if i == j%��ӷ����Ժ�ʽ�����б仯�����Խ��Ӱ��Ҳ����
                F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
            else
                F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
            end
            G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
        end
    end
    Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
    Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
    A = [O,I;F+Fe,G+Ge];    % ϵ������
    
    %��ֵ����
    [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
end
es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % ǰһʱ�̣�1��ʱ�̵ı�ƹ켣���Ϻ�һʱ��(2)ʱ�̵�ʱ���ݻ���������״̬�������ݻ�
%% Э�����������������켣
% for t = [540 900 1260 end_num-200]
for t = 1:1800
    %      hold on ;     %hold on�ڵ�ǰ���ϱ���ͼ�������ӵ����ϵ���ͼ����ɾ������ͼ��
    %     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
    %     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
    
    zeta2 = chi2inv(0.99999,3);   % ���Ͼ���
    zeta = sqrt(zeta2);
    
    C=[I,O];      % λ�õľ�ֵ��Э�������
    ex=C*mean(es(6*(t-1)+1:t*6,:)')';
    X=C*cov(es(6*(t-1)+1:t*6,:)')*C';
    [U,Lam] = eig(X) ;
    
    %  eig���Ƶ�����ֵ���Ȳ�������������Э��������ǰ�������������ֵ>=0��һ����
    %  ���������С��0������ֵȡ0
    for i = 1:size(Lam)
        for j =1:size(Lam)
            if(Lam(i,j)<0)
                Lam(i,j)=0;
            end
        end
    end
  %% �ڹ�ʽ53����lambda��alpha���ȡ��͵ȼ��ȡ�����������ֲ�����仯�ϴ� 
    %�ȼ��ȡ��
    lambda = linspace(-pi/2,pi/2,col);
    alpha = linspace(0,2*pi,col);
    %���ȡ��
%     lambda =pi*rand([1,col])-(pi/2);   %����ֲ�դ��,rand([1,col])��ʾ1�У�col����0��1������ȡ�����
%     alpha=2*pi*rand([1,row]);
  %%
    for i=1:col
        for j=1:row
            y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
            x=ex+U*Lam^(1/2)*y;
            x=real(x)/1000;
            a=(i-1)*col+j;
            xx(a,:)=x;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','o','color','red','MarkerSize',1);
            suipian1=[suipian1;x(1),x(2),t];
            
        end
    end
  
    area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xlswrite('E:\2021�о������д�����Ŀ--��������\��ƬΣ����+A���㷨\С���� �ǹ�� ����\suipian_chushi.xlsx',suipian1);
    
    
%     %% ��ά͹�������ʾ
%     % figure;
%     %conhull�����ķ���ֵΪ͹�ͱ߽�ĵ���������,kΪ��ʱ�����ӱ߽��ʱ�ĵ��ھ�����λ�ñ��.
%     [k area]= convhull(xx(:,1),xx(:,2));
%     hold on;
%     aver=sum(xx(:,3))/NN;     %����NN����߶�ȡ��ֵ.
%     xx(:,3)=aver;
%     plot3(xx(:,1),xx(:,2),xx(:,3), '.', 'markersize',8);
%     plot3(xx(k,1), xx(k,2),xx(k,3),'LineWidth',1); % ��ʱ�����������߽��
%     set(gca, 'XLim',[min(xx(k,1))-80 max(xx(k,1))+80]); % X���������ʾ��Χ
%     set(gca, 'YLim',[min(xx(k,2))-20 max(xx(k,2))+20]);
%     grid on;
end
% for t = 540
%     hold on ;     %hold on�ڵ�ǰ���ϱ���ͼ�������ӵ����ϵ���ͼ����ɾ������ͼ��
%     plot3(z_stand(6*(t-1)+1,1),z_stand(6*(t-1)+2,1),z_stand(6*(t-1)+3,1),'LineStyle','none', 'Marker','.','markersize',7,'MarkerEdgeColor','red');
%     text(x_nt(1,t),x_nt(2,t)+15,x_nt(3,t),['t=' num2str(t/60),'min']);
% end
% 
% for t = 900
%     hold on ;     %hold on�ڵ�ǰ���ϱ���ͼ�������ӵ����ϵ���ͼ����ɾ������ͼ��
%     plot3(z_stand(6*(t-1)+1,1),z_stand(6*(t-1)+2,1),z_stand(6*(t-1)+3,1),'LineStyle','none', 'Marker','.','markersize',7,'MarkerEdgeColor','red');
%     text(x_nt(1,t)+5,x_nt(2,t)+15,x_nt(3,t)-1,['t=' num2str(t/60),'min']);
% end
% 
% for t = 1260
%     hold on ;     %hold on�ڵ�ǰ���ϱ���ͼ�������ӵ����ϵ���ͼ����ɾ������ͼ��
%     plot3(z_stand(6*(t-1)+1,1),z_stand(6*(t-1)+2,1),z_stand(6*(t-1)+3,1),'LineStyle','none', 'Marker','.','markersize',7,'MarkerEdgeColor','red');
%     text(x_nt(1,t)+5,x_nt(2,t)+15,x_nt(3,t)-1,['t=' num2str(t/60),'min']);
% end
% for t = end_num-200
%     hold on ;     %hold on�ڵ�ǰ���ϱ���ͼ�������ӵ����ϵ���ͼ����ɾ������ͼ��
%     plot3(z_stand(6*(t-1)+1,1),z_stand(6*(t-1)+2,1),z_stand(6*(t-1)+3,1),'LineStyle','none', 'Marker','.','markersize',7,'MarkerEdgeColor','red');
%     text(x_nt(1,t)+5,x_nt(2,t)+15,x_nt(3,t)-1,['t=' num2str(t/60),'min']);
% end

   %% ��xx����ȥ��������ͬ������Ϣ�ĵ�
% xx=roundn(xx,-4);
% Index=[];
% q=1;
%     for i=1:Point^2
%         for j=(i+1):Point^2
%             if xx(i,:)==xx(j,:)
%                 Index(q)=j;
%                 q=q+1;
%             end
%         end
%     end
% 
%     xx([Index],:)=[];
%% ��xx����ȥ��������ͬ������Ϣ�ĵ�
% suipian1=roundn(suipian1,-4);
suipian1=round(suipian1);
Index=[];
q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if suipian1(i,:)==suipian1(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end

    suipian1([Index],:)=[];
     %�õ���չ�������е������
   xVector=100:1:450;
   yVector=-40:1:20;
   zVector=1:1:1800;
 
   [x,y,z]=meshgrid(xVector,yVector,zVector); % obtain all coordinates
   
   in=inpolygon(x,y,suipian1(:,1),suipian1(:,2));%��չǰ
%    in=inpolygon(x,y,xx(:,1),xx(:,2));
   inX=x(in);
   inY=y(in);
   suipian=[inX,inY];
% xlswrite('E:\2021�о������д�����Ŀ--��������\��ƬΣ����+A���㷨\С���� �ǹ�� ����\suipian_quchong.xlsx',suipian1);
xlswrite('E:\2021�о������д�����Ŀ--��������\��ƬΣ����+A���㷨\С���� �ǹ�� ����\suipian_quchong_zhengshu.xlsx',suipian1);
%% ��ά���һ��ʱ��͹����ͱ߽���ʾ 
%      figure;
%     %     [k area]= convhull(xx(:,1),xx(:,2));
%     [k area]= convhull(xx(:,1),xx(:,2))
%     hold on;
%     xlabel('x/km');
%     ylabel('y/km');
%     plot(xx(:,1),xx(:,2), '.', 'markersize',10,'HandleVisibility','off');
%     plot(xx(k,1), xx(k,2),'-','LineWidth',1.5,'HandleVisibility','off'); % ��ʱ�����������߽��
% %     legend('��ʼ�߽�');
%     set(gca, 'XLim',[min(xx(k,1))-60 max(xx(k,1))+60]); % X���������ʾ��Χ
%     set(gca, 'YLim',[min(xx(k,2))-10 max(xx(k,2))+10]);
%     grid on;
%     hold on;
%% ��ά���һ��ʱ��͹����ͱ߽���ʾ 
     figure;
    %     [k area]= convhull(xx(:,1),xx(:,2));
    [k area]= convhull(suipian1(:,1),suipian1(:,2))
%     hold on;
%     xlabel('x/km');
%     ylabel('y/km');
%     plot(suipian1(:,1),suipian1(:,2), '.', 'markersize',10,'HandleVisibility','off');
%     plot(suipian1(k,1), suipian1(k,2),'-','LineWidth',1.5,'HandleVisibility','off'); % ��ʱ�����������߽��
% %     legend('��ʼ�߽�');
%     set(gca, 'XLim',[min(suipian1(k,1))-60 max(suipian1(k,1))+60]); % X���������ʾ��Χ
%     set(gca, 'YLim',[min(suipian1(k,2))-10 max(suipian1(k,2))+10]);
%     grid on;
%     hold on;



    %% ����·������
%     %��·��
%     q_0=[500 ,-20];
%     r_18=[440.4,-26.81];
%     r_17=[419,-29.48];
%     r_16=[388.4,-31.1];
%     r_15=[351.2,-31.63];
%     r_14=[311.7,-31.04];
%     r_13=[274.6,-29.38];
%     r_12=[244.1,-26.84];
%     r_11=[222.7,-23.52];
%     q_f=[140,-15];
%     
%     d_1=norm(q_0 - r_18);
%     d_2=norm(r_18 - r_17);
%     d_3=norm(r_17 - r_16);
%     d_4=norm(r_16 - r_15);
%     d_5=norm(r_15 - r_14);
%     d_6=norm(r_14 - r_13);
%     d_7=norm(r_13 - r_12);
%     d_8=norm(r_12 - r_11);
%     d_9=norm(r_11 - q_f);
%     sum_1= d_1+ d_2+ d_3+ d_4+ d_5+ d_6+ d_7+ d_8+ d_9;
%     % ��·��
%     q_0=[500 ,-20];
%     r_1=[468.5,-14.41];
%     r_2=[440.8,-1.167];
%     r_3=[419.4,2.155];
%     r_4=[388.9,4.691];
%     r_5=[351.8,6.346];
%     r_6=[312.3,6.942];
%     r_7=[275.1,6.409];
%     r_8=[244.5,4.795];
%     r_9=[223.1,2.119];
%     r_10=[195,-10.28];
%     q_f=[140,-15];
%    
%     d_10=norm(q_0 - r_1);
%     d_11=norm(r_1 - r_2);
%     d_12=norm(r_2 - r_3);
%     d_13=norm(r_3 - r_4);
%     d_14=norm(r_4 - r_5);
%     d_15=norm(r_5 - r_6);
%     d_16=norm(r_6 - r_7);
%     d_17=norm(r_7 - r_8);
%     d_18=norm(r_8 - r_9);
%     d_19=norm(r_9 - r_10);
%     d_20=norm(r_10 - q_f);
%     sum_2= d_10+ d_11+ d_12+ d_13+ d_14+ d_15+ d_16+ d_17+ d_18+d_19+ d_20;

   
   %% ��չ10km
%    % ������kֵӰ��expand�������ȥ���һ��
%    hold on;
%    p=[xx(k,1), xx(k,2)]; 
%    ii=size(k,1);% k������
%    [newp2]=expand(p,ii);
%    plot(newp2(:,1),newp2(:,2),'k-','LineWidth',1.5,'HandleVisibility','off');
%    set(gca, 'XLim',[min(p(:,1))-80 max(p(:,1))+80]); % X���������ʾ��Χ
%    set(gca, 'YLim',[min(p(:,2))-20 max(xx(:,2))+20]);
%    
%    fill(newp2(:,1),newp2(:,2),[0.5 0.5 0.5]);
%    fill(xx(k,1), xx(k,2),'k');
%    %�õ���չ�������е������
%    xVector=100:1:450;
%    yVector=-40:1:20;
%  
%    [x,y]=meshgrid(xVector,yVector); % obtain all coordinates
%    
%    in=inpolygon(x,y,newp2(:,1),newp2(:,2));
% %    in=inpolygon(x,y,xx(:,1),xx(:,2));
%    inX=x(in);
%    inY=y(in);
%    suipian=[inX,inY];
 %% ��չ10km
   % ������kֵӰ��expand�������ȥ���һ��
   hold on;
   p=[suipian1(k,1), suipian1(k,2)]; 
   ii=size(k,1);% k������
   [newp2]=expand(p,ii);
   plot(newp2(:,1),newp2(:,2),'k-','LineWidth',1.5,'HandleVisibility','off');
   set(gca, 'XLim',[min(p(:,1))-80 max(p(:,1))+80]); % X���������ʾ��Χ
   set(gca, 'YLim',[min(p(:,2))-20 max(suipian1(:,2))+20]);
   
   fill(newp2(:,1),newp2(:,2),[0.5 0.5 0.5]);
   fill(suipian1(k,1), suipian1(k,2),'k');
%    %�õ���չ�������е������
%    xVector=100:1:450;
%    yVector=-40:1:20;
%  
%    [x,y]=meshgrid(xVector,yVector); % obtain all coordinates
%    
% %    in=inpolygon(x,y,newp2(:,1),newp2(:,2));%��չ��
%    in=inpolygon(x,y,suipian1(:,1),suipian1(:,2));%��չǰ
% %    in=inpolygon(x,y,xx(:,1),xx(:,2));
%    inX=x(in);
%    inY=y(in);
%    suipian=[inX,inY];