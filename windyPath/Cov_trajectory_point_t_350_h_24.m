function suipian = Cov_trajectory_point_t_350_h_24()
% clc;
% close all;
% clear;

%%初始值设定
beta = 3.7;     
Delta_v = 55.64;                           %Δv(m/s)

z_0 = 75*1000;                            %(m)初始高度
psi0 = convang(0,'deg','rad');                                %(°)heading angle
gamma0 = convang(-1,'deg','rad');        %(°)flight path angle
v0=7.3*1000;                             %(m/s) relative speed

Point=10;
col=Point;
row=Point;
NN= col*row;

O = zeros(3);
I = eye(3);
B=[O;I];
vt0 = zeros(3,NN);

theta0 = convang(100.2,'deg','rad');      %(°W)经度
phi0 = convang(40.9,'deg','rad');          %(°N)纬度
                           %(kg/m2)弹道系数（仅对一个弹道系数进行研究）
%地球参量
omegae = 72.9217*10^(-6);                %自转角速度
re = 6372.8*1000;                        %(m)地球半径
ge = 9.81;                               %(m/s^2)地面重力加速度

% 初始加速度方向，随机取点
% phi =pi*rand([1,col])-(pi/2);   %椭球分布栅格,rand([1,col])表示1行，col列在0到1区间内取随机数
% theta =2*pi*rand([1,row]);

% 初始加速度方向，等间隔取点
phi = linspace(-pi/2,pi/2,col);
theta = linspace(0,2*pi,col);             % 0-180是东经，180-360对应着西经的180-0

V0 = diag([0.0025,0.0025,0.0053]);       %初速度矢量协方差矩阵

N=4000;                                 %最大迭代次数
%初始化
x1=zeros(1,N);
x2=zeros(1,N);
x3=zeros(1,N);
v1=zeros(1,N);
v2=zeros(1,N);
v3=zeros(1,N);

%平均初速度分量
x1(1)=0;    % 爆炸前一时刻的位置
x2(1)=0;
x3(1)=z_0;

v1(1)=v0*cos(gamma0)*cos(psi0);   % 爆炸前一时刻的速度
v2(1)=v0*cos(gamma0)*sin(psi0);
v3(1)=v0*sin(gamma0);

%初始位置矢量和速度矢量
x0e=[x1(1),x2(1),x3(1)]';
v0e = [v1(1),v2(1),v3(1)]';

ez=zeros(6*N,NN);    % N迭代次数
Ez=zeros(6*N,1);    % N迭代次数
Z=zeros(6*N,6);
z_stand = zeros(6*N,NN); %存储标称运动状态
%% 描述标称轨迹
% 以飞机解体前的状态（3个位置分量和3个速度分量），带入运动方程10-15，求出标称轨迹。
for n=1:N-1
    % w=[0,0,0]'; %假设风为0
    z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
    w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0时刻，实施水平风模型
    
    % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%大气经验密度模型MSISE-00（质谱仪非相干散射雷达扩展2000），T温度，rho密度
    rhoe = 1.752;                         %(kg/m^3)
    H = 6.7*1000;                              %(km)
    rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);   
    h=1;
    g = ge*(re/(re+x3(n)))^2;     % 逆方重力模型
    vn=[v1(n),v2(n),v3(n)]';      % 
    va=vn-w;
    va_norm=norm(va); %求空气速率的模
    %三维运动方程
    [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
    stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %核心公式
    end_num=n;   
    if (x3(n+1)<0)
       end_num=n-1;
       break;
    end

end
z_stand = repmat(z_stand(:,1),[1,NN]);

%% 标称轨迹画图
v1_r=v1(1:end_num);
v2_r=v2(1:end_num);
v3_r=v3(1:end_num);
x1_r=x1(1:end_num)/1000;  %仿真图坐标轴单位为km
x2_r=x2(1:end_num)/1000;
x3_r=x3(1:end_num)/1000;

%标称轨迹坐标，就是仿真图中的轨迹
%通过推导三维平动运动方程，以破碎前的状态向量为初始状态，以平均弹道系数为基准，得到了一条标称弹道。
x_nt=[x1_r;x2_r;x3_r];       %文献中（26）   标称轨迹表达式,3×1799
v_nt=[v1_r;v2_r;v3_r]; 

% % plot3(x1_r,x2_r,x3_r,'LineWidth',1);       % 在3D空间中绘制画出标称轨迹x_nt
% % view([55 32]);               % 将方位角设置为55度，将高程设置为32度
% % set(gca,'YDir','reverse');   % 将Y轴反过来
% set(gca, 'XLim',[0 500]);   % X轴的数据显示范围
% set(gca, 'YLim',[-30 10]); 
% % set(gca, 'ZLim',[0 80]);    
% 
% xlabel('x1/km');
% ylabel('x2/km');
% % zlabel('x3/km');
% grid on;
% % set(gca, 'GridLineStyle', ':');  % 设置为虚线
% % title('18*18点协方差算法(有风)');

%% 各个方向碎片初始速度(NN个（样本点）轨迹初速度)
for i = 1:col
    for j = 1:row
       
        vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
        sn0 = [x0e',v0e']';  % s*0标称轨迹
        s0 = [x0e',vt0(:,(i-1)*col+j)']';  % 实际状态向量
        
        z0 = s0-sn0;    % 实际 - 理论 = 偏差，研究偏差。实际不确定性，波动，随机过程，通过协方差来研究（核心方法）
        Z0 = [O,O;O,V0];     % 协方差矩阵
        ez(1:6,(i-1)*col+j) = z0;   % NN个样本点求均值的初始条件    !!!!!
        Z(1:6,1:6)=Z0;    % 存储协方差矩阵
    end
end

%% 求碎片的运动散布
 for tn = 1:end_num
%         h = 1; 
        if tn > 1 %用经向纬向风速随高度变化的变化速度作为偏导
            w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
            hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
            hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
            dw = (w2-w1)./2/hw'; %d高
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        else
            dw = [0.0026 0.0015 0];
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        end

        g = ge*(re/(re+x3(n)))^2;     %逆方重力模型
        vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %当前标称v(123)
        va = vn - w;
        va_norm = norm(va);  %相对空气速度的大小
        
        rhoe = 1.752;   %(kg/m^3)
        H = 6.7*1000;   %(km)
        rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %大气密度@该海拔高度（x3）
        drho1 = 0;
        drho2 = 0;     %p对x2求偏导
        drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %公式（17）,这是p对x3求偏导
        drho = [drho1,drho2 drho3]';
        
        % 时变系数矩阵A(t);
        F = zeros(3);   %3*3的零矩阵，初始化
        G = zeros(3);
        delta = eye(3);
        
        for i = 1:3
            for j = 1:3
                if i == j%添加风速以后，式子略有变化，但对结果影响也不大。
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
                else
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
                end
                G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
            end
        end
        Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
        Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
        A = [O,I;F+Fe,G+Ge];    % 系数矩阵
        
        %数值积分
        [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
 end
        es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % 前一时刻（1）时刻的标称轨迹加上后一时刻(2)时刻的时间演化等于所有状态变量的演化
%% 协方差传播方法描述椭球轨迹
for t = [ 350 ]
    hold on ;     %hold on在当前轴上保留图，因此添加到轴上的新图不会删除现有图。
%     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
%     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
    
    zeta2 = chi2inv(0.99999,3);   % 马氏距离
    zeta = sqrt(zeta2);
    
    C=[I,O];      % 位置的均值和协方差矩阵
    ex=C*mean(es(6*(t-1)+1:t*6,:)')';  
    X=C*cov(es(6*(t-1)+1:t*6,:)')*C'; 
    [U,Lam] = eig(X) ;
    
    %  eig估计的特征值精度不够，根据样本协方差矩阵是半正定矩阵（特征值>=0这一特性
    %  把求出的略小于0的特征值取0
    for i = 1:size(Lam)
        for j =1:size(Lam)
            if(Lam(i,j)<0)
                Lam(i,j)=0;
            end
        end
    end
    %等间隔取点
    lambda = linspace(-pi/2,pi/2,col);
    alpha = linspace(0,2*pi,col);
    %随机取点
%      lambda =pi*rand([1,col])-pi/2;   %椭球分布栅格 
%      alpha=2*pi*rand([1,row]); 

     for i=1:col
         for j=1:row
             y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
             x=ex+U*Lam^(1/2)*y;
             x=real(x)/1000;
            a=(i-1)*col+j;
            xx3(a,:)=x;
%             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','.','color','m','MarkerSize',6);
         end
     end
%      xx=xx-z_stand(6*(t-1)+1:6*(t-1)+3,1)'/1e3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


xx3=roundn(xx3,-4);
Index=[];
q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if xx3(i,:)==xx3(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end

    xx3([Index],:)=[];



    
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    %初始值设定
    beta = 4.4;
    Delta_v = 53.65;                           %Δv(m/s)
   
    z_0 = 75*1000;                            %(m)初始高度
    psi0 = convang(0,'deg','rad');                                %(°)heading angle
    gamma0 = convang(-1,'deg','rad');        %(°)flight path angle
    v0=7.3*1000;                             %(m/s) relative speed
    
    Point=10;
    col=Point;
    row=Point;
    NN= col*row;
    
    O = zeros(3);
    I = eye(3);
    B=[O;I];
    vt0 = zeros(3,NN);
    
    theta0 = convang(100.2,'deg','rad');      %(°W)经度
    phi0 = convang(40.9,'deg','rad');          %(°N)纬度
    %(kg/m2)弹道系数（仅对一个弹道系数进行研究）
    %地球参量
    omegae = 72.9217*10^(-6);                %自转角速度
    re = 6372.8*1000;                        %(m)地球半径
    ge = 9.81;                               %(m/s^2)地面重力加速度
    
    % 初始加速度方向，随机取点
    % phi =pi*rand([1,col])-(pi/2);   %椭球分布栅格,rand([1,col])表示1行，col列在0到1区间内取随机数
    % theta =2*pi*rand([1,row]);
    
    % 初始加速度方向，等间隔取点
    phi = linspace(-pi/2,pi/2,col);
    theta = linspace(0,2*pi,col);             % 0-180是东经，180-360对应着西经的180-0
    
    V0 = diag([0.0025,0.0025,0.0053]);       %初速度矢量协方差矩阵
    
    N=4000;                                 %最大迭代次数
    %初始化
    x1=zeros(1,N);
    x2=zeros(1,N);
    x3=zeros(1,N);
    v1=zeros(1,N);
    v2=zeros(1,N);
    v3=zeros(1,N);
    
    %平均初速度分量
    x1(1)=0;    % 爆炸前一时刻的位置
    x2(1)=0;
    x3(1)=z_0;
    
    v1(1)=v0*cos(gamma0)*cos(psi0);   % 爆炸前一时刻的速度
    v2(1)=v0*cos(gamma0)*sin(psi0);
    v3(1)=v0*sin(gamma0);
    
    %初始位置矢量和速度矢量
    x0e=[x1(1),x2(1),x3(1)]';
    v0e = [v1(1),v2(1),v3(1)]';
    
    ez=zeros(6*N,NN);    % N迭代次数
    Ez=zeros(6*N,1);    % N迭代次数
    Z=zeros(6*N,6);
    z_stand = zeros(6*N,NN); %存储标称运动状态
    %% 描述标称轨迹
    % 以飞机解体前的状态（3个位置分量和3个速度分量），带入运动方程10-15，求出标称轨迹。
    for n=1:N-1
        % w=[0,0,0]'; %假设风为0
        z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
        w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0时刻，实施水平风模型
        
        % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%大气经验密度模型MSISE-00（质谱仪非相干散射雷达扩展2000），T温度，rho密度
        rhoe = 1.752;                         %(kg/m^3)
        H = 6.7*1000;                              %(km)
        rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);
        h=1;
        g = ge*(re/(re+x3(n)))^2;     % 逆方重力模型
        vn=[v1(n),v2(n),v3(n)]';      %
        va=vn-w;
        va_norm=norm(va); %求空气速率的模
        %三维运动方程
        [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
            stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %核心公式
        end_num=n;
        if (x3(n+1)<0)
            end_num=n-1;
            break;
        end
        
    end
    z_stand = repmat(z_stand(:,1),[1,NN]);
    
    %% 标称轨迹画图
    v1_r=v1(1:end_num);
    v2_r=v2(1:end_num);
    v3_r=v3(1:end_num);
    x1_r=x1(1:end_num)/1000;  %仿真图坐标轴单位为km
    x2_r=x2(1:end_num)/1000;
    x3_r=x3(1:end_num)/1000;
    
    %标称轨迹坐标，就是仿真图中的轨迹
    %通过推导三维平动运动方程，以破碎前的状态向量为初始状态，以平均弹道系数为基准，得到了一条标称弹道。
    x_nt=[x1_r;x2_r;x3_r];       %文献中（26）   标称轨迹表达式,3×1799
    v_nt=[v1_r;v2_r;v3_r];
    
%     % plot3(x1_r,x2_r,x3_r,'LineWidth',1);       % 在3D空间中绘制画出标称轨迹x_nt
% %     view([55 32]);               % 将方位角设置为55度，将高程设置为32度
%     % set(gca,'YDir','reverse');   % 将Y轴反过来
%     set(gca, 'XLim',[0 500]);   % X轴的数据显示范围
%     set(gca, 'YLim',[-30 10]);
% %     set(gca, 'ZLim',[0 80]);
%     
%     xlabel('x1/km');
%     ylabel('x2/km');
% %     zlabel('x3/km');
%     grid on;
%     % set(gca, 'GridLineStyle', ':');  % 设置为虚线
%     % title('18*18点协方差算法(有风)');
    
    %% 各个方向碎片初始速度(NN个（样本点）轨迹初速度)
    for i = 1:col
        for j = 1:row
            
            vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
            sn0 = [x0e',v0e']';  % s*0标称轨迹
            s0 = [x0e',vt0(:,(i-1)*col+j)']';  % 实际状态向量
            
            z0 = s0-sn0;    % 实际 - 理论 = 偏差，研究偏差。实际不确定性，波动，随机过程，通过协方差来研究（核心方法）
            Z0 = [O,O;O,V0];     % 协方差矩阵
            ez(1:6,(i-1)*col+j) = z0;   % NN个样本点求均值的初始条件    !!!!!
            Z(1:6,1:6)=Z0;    % 存储协方差矩阵
        end
    end
    
    %% 求碎片的运动散布
    for tn = 1:end_num
        %         h = 1;
        if tn > 1 %用经向纬向风速随高度变化的变化速度作为偏导
            w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
            hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
            hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
            dw = (w2-w1)./2/hw'; %d高
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        else
            dw = [0.0026 0.0015 0];
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        end
        
        g = ge*(re/(re+x3(n)))^2;     %逆方重力模型
        vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %当前标称v(123)
        va = vn - w;
        va_norm = norm(va);  %相对空气速度的大小
        
        rhoe = 1.752;   %(kg/m^3)
        H = 6.7*1000;   %(km)
        rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %大气密度@该海拔高度（x3）
        drho1 = 0;
        drho2 = 0;     %p对x2求偏导
        drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %公式（17）,这是p对x3求偏导
        drho = [drho1,drho2 drho3]';
        
        % 时变系数矩阵A(t);
        F = zeros(3);   %3*3的零矩阵，初始化
        G = zeros(3);
        delta = eye(3);
        
        for i = 1:3
            for j = 1:3
                if i == j%添加风速以后，式子略有变化，但对结果影响也不大。
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
                else
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
                end
                G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
            end
        end
        Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
        Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
        A = [O,I;F+Fe,G+Ge];    % 系数矩阵
        
        %数值积分
        [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
    end
    es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % 前一时刻（1）时刻的标称轨迹加上后一时刻(2)时刻的时间演化等于所有状态变量的演化
    %% 协方差传播方法描述椭球轨迹
    for t = [ 350 ]
        hold on ;     %hold on在当前轴上保留图，因此添加到轴上的新图不会删除现有图。
        %     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
        %     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
        
        zeta2 = chi2inv(0.99999,3);   % 马氏距离
        zeta = sqrt(zeta2);
        
        C=[I,O];      % 位置的均值和协方差矩阵
        ex=C*mean(es(6*(t-1)+1:t*6,:)')';
        X=C*cov(es(6*(t-1)+1:t*6,:)')*C';
        [U,Lam] = eig(X) ;
        
        %  eig估计的特征值精度不够，根据样本协方差矩阵是半正定矩阵（特征值>=0这一特性
        %  把求出的略小于0的特征值取0
        for i = 1:size(Lam)
            for j =1:size(Lam)
                if(Lam(i,j)<0)
                    Lam(i,j)=0;
                end
            end
        end
        %等间隔取点
        lambda = linspace(-pi/2,pi/2,col);
        alpha = linspace(0,2*pi,col);
        %随机取点
        %      lambda =pi*rand([1,col])-pi/2;   %椭球分布栅格
        %      alpha=2*pi*rand([1,row]);
        
        for i=1:col
            for j=1:row
                y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
                x=ex+U*Lam^(1/2)*y;
                x=real(x)/1000;
                a=(i-1)*col+j;
                xx4(a,:)=x;
                %             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','.','color','m','MarkerSize',6);
            end
        end
        %      xx=xx-z_stand(6*(t-1)+1:6*(t-1)+3,1)'/1e3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    xx4=roundn(xx4,-4);
    Index=[];
    q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if xx4(i,:)==xx4(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end
    
    xx4([Index],:)=[];
 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
     %初始值设定
    beta = 6.5;
    Delta_v = 49.2;                           %Δv(m/s)
    
    z_0 = 75*1000;                            %(m)初始高度
    psi0 = convang(0,'deg','rad');                                %(°)heading angle
    gamma0 = convang(-1,'deg','rad');        %(°)flight path angle
    v0=7.3*1000;                             %(m/s) relative speed
    
    Point=10;
    col=Point;
    row=Point;
    NN= col*row;
    
    O = zeros(3);
    I = eye(3);
    B=[O;I];
    vt0 = zeros(3,NN);
    
    theta0 = convang(100.2,'deg','rad');      %(°W)经度
    phi0 = convang(40.9,'deg','rad');          %(°N)纬度
    %(kg/m2)弹道系数（仅对一个弹道系数进行研究）
    %地球参量
    omegae = 72.9217*10^(-6);                %自转角速度
    re = 6372.8*1000;                        %(m)地球半径
    ge = 9.81;                               %(m/s^2)地面重力加速度
    
    % 初始加速度方向，随机取点
    % phi =pi*rand([1,col])-(pi/2);   %椭球分布栅格,rand([1,col])表示1行，col列在0到1区间内取随机数
    % theta =2*pi*rand([1,row]);
    
    % 初始加速度方向，等间隔取点
    phi = linspace(-pi/2,pi/2,col);
    theta = linspace(0,2*pi,col);             % 0-180是东经，180-360对应着西经的180-0
    
    V0 = diag([0.0025,0.0025,0.0053]);       %初速度矢量协方差矩阵
    
    N=4000;                                 %最大迭代次数
    %初始化
    x1=zeros(1,N);
    x2=zeros(1,N);
    x3=zeros(1,N);
    v1=zeros(1,N);
    v2=zeros(1,N);
    v3=zeros(1,N);
    
    %平均初速度分量
    x1(1)=0;    % 爆炸前一时刻的位置
    x2(1)=0;
    x3(1)=z_0;
    
    v1(1)=v0*cos(gamma0)*cos(psi0);   % 爆炸前一时刻的速度
    v2(1)=v0*cos(gamma0)*sin(psi0);
    v3(1)=v0*sin(gamma0);
    
    %初始位置矢量和速度矢量
    x0e=[x1(1),x2(1),x3(1)]';
    v0e = [v1(1),v2(1),v3(1)]';
    
    ez=zeros(6*N,NN);    % N迭代次数
    Ez=zeros(6*N,1);    % N迭代次数
    Z=zeros(6*N,6);
    z_stand = zeros(6*N,NN); %存储标称运动状态
    %% 描述标称轨迹
    % 以飞机解体前的状态（3个位置分量和3个速度分量），带入运动方程10-15，求出标称轨迹。
    for n=1:N-1
        % w=[0,0,0]'; %假设风为0
        z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
        w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0时刻，实施水平风模型
        
        % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%大气经验密度模型MSISE-00（质谱仪非相干散射雷达扩展2000），T温度，rho密度
        rhoe = 1.752;                         %(kg/m^3)
        H = 6.7*1000;                              %(km)
        rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);
        h=1;
        g = ge*(re/(re+x3(n)))^2;     % 逆方重力模型
        vn=[v1(n),v2(n),v3(n)]';      %
        va=vn-w;
        va_norm=norm(va); %求空气速率的模
        %三维运动方程
        [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
            stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %核心公式
        end_num=n;
        if (x3(n+1)<0)
            end_num=n-1;
            break;
        end
        
    end
    z_stand = repmat(z_stand(:,1),[1,NN]);
    
    %% 标称轨迹画图
    v1_r=v1(1:end_num);
    v2_r=v2(1:end_num);
    v3_r=v3(1:end_num);
    x1_r=x1(1:end_num)/1000;  %仿真图坐标轴单位为km
    x2_r=x2(1:end_num)/1000;
    x3_r=x3(1:end_num)/1000;
    
    %标称轨迹坐标，就是仿真图中的轨迹
    %通过推导三维平动运动方程，以破碎前的状态向量为初始状态，以平均弹道系数为基准，得到了一条标称弹道。
    x_nt=[x1_r;x2_r;x3_r];       %文献中（26）   标称轨迹表达式,3×1799
    v_nt=[v1_r;v2_r;v3_r];
    
%     % plot3(x1_r,x2_r,x3_r,'LineWidth',1);       % 在3D空间中绘制画出标称轨迹x_nt
% %     view([55 32]);               % 将方位角设置为55度，将高程设置为32度
%     % set(gca,'YDir','reverse');   % 将Y轴反过来
%     set(gca, 'XLim',[0 500]);   % X轴的数据显示范围
%     set(gca, 'YLim',[-30 10]);
% %     set(gca, 'ZLim',[0 80]);
%     
%     xlabel('x1/km');
%     ylabel('x2/km');
% %     zlabel('x3/km');
%     grid on;
%     % set(gca, 'GridLineStyle', ':');  % 设置为虚线
%     % title('18*18点协方差算法(有风)');
    
    %% 各个方向碎片初始速度(NN个（样本点）轨迹初速度)
    for i = 1:col
        for j = 1:row
            
            vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
            sn0 = [x0e',v0e']';  % s*0标称轨迹
            s0 = [x0e',vt0(:,(i-1)*col+j)']';  % 实际状态向量
            
            z0 = s0-sn0;    % 实际 - 理论 = 偏差，研究偏差。实际不确定性，波动，随机过程，通过协方差来研究（核心方法）
            Z0 = [O,O;O,V0];     % 协方差矩阵
            ez(1:6,(i-1)*col+j) = z0;   % NN个样本点求均值的初始条件    !!!!!
            Z(1:6,1:6)=Z0;    % 存储协方差矩阵
        end
    end
    
    %% 求碎片的运动散布
    for tn = 1:end_num
        %         h = 1;
        if tn > 1 %用经向纬向风速随高度变化的变化速度作为偏导
            w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
            hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
            hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
            dw = (w2-w1)./2/hw'; %d高
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        else
            dw = [0.0026 0.0015 0];
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        end
        
        g = ge*(re/(re+x3(n)))^2;     %逆方重力模型
        vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %当前标称v(123)
        va = vn - w;
        va_norm = norm(va);  %相对空气速度的大小
        
        rhoe = 1.752;   %(kg/m^3)
        H = 6.7*1000;   %(km)
        rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %大气密度@该海拔高度（x3）
        drho1 = 0;
        drho2 = 0;     %p对x2求偏导
        drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %公式（17）,这是p对x3求偏导
        drho = [drho1,drho2 drho3]';
        
        % 时变系数矩阵A(t);
        F = zeros(3);   %3*3的零矩阵，初始化
        G = zeros(3);
        delta = eye(3);
        
        for i = 1:3
            for j = 1:3
                if i == j%添加风速以后，式子略有变化，但对结果影响也不大。
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
                else
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
                end
                G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
            end
        end
        Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
        Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
        A = [O,I;F+Fe,G+Ge];    % 系数矩阵
        
        %数值积分
        [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
    end
    es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % 前一时刻（1）时刻的标称轨迹加上后一时刻(2)时刻的时间演化等于所有状态变量的演化
    %% 协方差传播方法描述椭球轨迹
    for t = [ 350 ]
        hold on ;     %hold on在当前轴上保留图，因此添加到轴上的新图不会删除现有图。
        %     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
        %     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
        
        zeta2 = chi2inv(0.99999,3);   % 马氏距离
        zeta = sqrt(zeta2);
        
        C=[I,O];      % 位置的均值和协方差矩阵
        ex=C*mean(es(6*(t-1)+1:t*6,:)')';
        X=C*cov(es(6*(t-1)+1:t*6,:)')*C';
        [U,Lam] = eig(X) ;
        
        %  eig估计的特征值精度不够，根据样本协方差矩阵是半正定矩阵（特征值>=0这一特性
        %  把求出的略小于0的特征值取0
        for i = 1:size(Lam)
            for j =1:size(Lam)
                if(Lam(i,j)<0)
                    Lam(i,j)=0;
                end
            end
        end
        %等间隔取点
        lambda = linspace(-pi/2,pi/2,col);
        alpha = linspace(0,2*pi,col);
        %随机取点
        %      lambda =pi*rand([1,col])-pi/2;   %椭球分布栅格
        %      alpha=2*pi*rand([1,row]);
        
        for i=1:col
            for j=1:row
                y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
                x=ex+U*Lam^(1/2)*y;
                x=real(x)/1000;
                a=(i-1)*col+j;
                xx6(a,:)=x;
                %             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','.','color','m','MarkerSize',6);
            end
        end
        %      xx=xx-z_stand(6*(t-1)+1:6*(t-1)+3,1)'/1e3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    xx6=roundn(xx6,-4);
    Index=[];
    q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if xx6(i,:)==xx6(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end
    
    xx6([Index],:)=[];
 
%     
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
         %初始值设定
    beta =7.3;
    Delta_v = 47.08;                           %Δv(m/s)
    
    z_0 = 75*1000;                            %(m)初始高度
    psi0 = convang(0,'deg','rad');                                %(°)heading angle
    gamma0 = convang(-1,'deg','rad');        %(°)flight path angle
    v0=7.3*1000;                             %(m/s) relative speed
    
    Point=10;
    col=Point;
    row=Point;
    NN= col*row;
    
    O = zeros(3);
    I = eye(3);
    B=[O;I];
    vt0 = zeros(3,NN);
    
    theta0 = convang(100.2,'deg','rad');      %(°W)经度
    phi0 = convang(40.9,'deg','rad');          %(°N)纬度
    %(kg/m2)弹道系数（仅对一个弹道系数进行研究）
    %地球参量
    omegae = 72.9217*10^(-6);                %自转角速度
    re = 6372.8*1000;                        %(m)地球半径
    ge = 9.81;                               %(m/s^2)地面重力加速度
    
    % 初始加速度方向，随机取点
    % phi =pi*rand([1,col])-(pi/2);   %椭球分布栅格,rand([1,col])表示1行，col列在0到1区间内取随机数
    % theta =2*pi*rand([1,row]);
    
    % 初始加速度方向，等间隔取点
    phi = linspace(-pi/2,pi/2,col);
    theta = linspace(0,2*pi,col);             % 0-180是东经，180-360对应着西经的180-0
    
    V0 = diag([0.0025,0.0025,0.0053]);       %初速度矢量协方差矩阵
    
    N=4000;                                 %最大迭代次数
    %初始化
    x1=zeros(1,N);
    x2=zeros(1,N);
    x3=zeros(1,N);
    v1=zeros(1,N);
    v2=zeros(1,N);
    v3=zeros(1,N);
    
    %平均初速度分量
    x1(1)=0;    % 爆炸前一时刻的位置
    x2(1)=0;
    x3(1)=z_0;
    
    v1(1)=v0*cos(gamma0)*cos(psi0);   % 爆炸前一时刻的速度
    v2(1)=v0*cos(gamma0)*sin(psi0);
    v3(1)=v0*sin(gamma0);
    
    %初始位置矢量和速度矢量
    x0e=[x1(1),x2(1),x3(1)]';
    v0e = [v1(1),v2(1),v3(1)]';
    
    ez=zeros(6*N,NN);    % N迭代次数
    Ez=zeros(6*N,1);    % N迭代次数
    Z=zeros(6*N,6);
    z_stand = zeros(6*N,NN); %存储标称运动状态
    %% 描述标称轨迹
    % 以飞机解体前的状态（3个位置分量和3个速度分量），带入运动方程10-15，求出标称轨迹。
    for n=1:N-1
        % w=[0,0,0]'; %假设风为0
        z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
        w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0时刻，实施水平风模型
        
        % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%大气经验密度模型MSISE-00（质谱仪非相干散射雷达扩展2000），T温度，rho密度
        rhoe = 1.752;                         %(kg/m^3)
        H = 6.7*1000;                              %(km)
        rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);
        h=1;
        g = ge*(re/(re+x3(n)))^2;     % 逆方重力模型
        vn=[v1(n),v2(n),v3(n)]';      %
        va=vn-w;
        va_norm=norm(va); %求空气速率的模
        %三维运动方程
        [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
            stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %核心公式
        end_num=n;
        if (x3(n+1)<0)
            end_num=n-1;
            break;
        end
        
    end
    z_stand = repmat(z_stand(:,1),[1,NN]);
    
    %% 标称轨迹画图
    v1_r=v1(1:end_num);
    v2_r=v2(1:end_num);
    v3_r=v3(1:end_num);
    x1_r=x1(1:end_num)/1000;  %仿真图坐标轴单位为km
    x2_r=x2(1:end_num)/1000;
    x3_r=x3(1:end_num)/1000;
    
    %标称轨迹坐标，就是仿真图中的轨迹
    %通过推导三维平动运动方程，以破碎前的状态向量为初始状态，以平均弹道系数为基准，得到了一条标称弹道。
    x_nt=[x1_r;x2_r;x3_r];       %文献中（26）   标称轨迹表达式,3×1799
    v_nt=[v1_r;v2_r;v3_r];
    
%     % plot3(x1_r,x2_r,x3_r,'LineWidth',1);       % 在3D空间中绘制画出标称轨迹x_nt
% %     view([55 32]);               % 将方位角设置为55度，将高程设置为32度
%     % set(gca,'YDir','reverse');   % 将Y轴反过来
%     set(gca, 'XLim',[0 500]);   % X轴的数据显示范围
%     set(gca, 'YLim',[-30 10]);
% %     set(gca, 'ZLim',[0 80]);
%     
%     xlabel('x1/km');
%     ylabel('x2/km');
% %     zlabel('x3/km');
%     grid on;
%     % set(gca, 'GridLineStyle', ':');  % 设置为虚线
%     % title('18*18点协方差算法(有风)');
    
    %% 各个方向碎片初始速度(NN个（样本点）轨迹初速度)
    for i = 1:col
        for j = 1:row
            
            vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
            sn0 = [x0e',v0e']';  % s*0标称轨迹
            s0 = [x0e',vt0(:,(i-1)*col+j)']';  % 实际状态向量
            
            z0 = s0-sn0;    % 实际 - 理论 = 偏差，研究偏差。实际不确定性，波动，随机过程，通过协方差来研究（核心方法）
            Z0 = [O,O;O,V0];     % 协方差矩阵
            ez(1:6,(i-1)*col+j) = z0;   % NN个样本点求均值的初始条件    !!!!!
            Z(1:6,1:6)=Z0;    % 存储协方差矩阵
        end
    end
    
    %% 求碎片的运动散布
    for tn = 1:end_num
        %         h = 1;
        if tn > 1 %用经向纬向风速随高度变化的变化速度作为偏导
            w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
            hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
            hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
            dw = (w2-w1)./2/hw'; %d高
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        else
            dw = [0.0026 0.0015 0];
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        end
        
        g = ge*(re/(re+x3(n)))^2;     %逆方重力模型
        vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %当前标称v(123)
        va = vn - w;
        va_norm = norm(va);  %相对空气速度的大小
        
        rhoe = 1.752;   %(kg/m^3)
        H = 6.7*1000;   %(km)
        rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %大气密度@该海拔高度（x3）
        drho1 = 0;
        drho2 = 0;     %p对x2求偏导
        drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %公式（17）,这是p对x3求偏导
        drho = [drho1,drho2 drho3]';
        
        % 时变系数矩阵A(t);
        F = zeros(3);   %3*3的零矩阵，初始化
        G = zeros(3);
        delta = eye(3);
        
        for i = 1:3
            for j = 1:3
                if i == j%添加风速以后，式子略有变化，但对结果影响也不大。
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
                else
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
                end
                G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
            end
        end
        Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
        Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
        A = [O,I;F+Fe,G+Ge];    % 系数矩阵
        
        %数值积分
        [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
    end
    es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % 前一时刻（1）时刻的标称轨迹加上后一时刻(2)时刻的时间演化等于所有状态变量的演化
    %% 协方差传播方法描述椭球轨迹
    for t = [ 350 ]
        hold on ;     %hold on在当前轴上保留图，因此添加到轴上的新图不会删除现有图。
        %     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
        %     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
        
        zeta2 = chi2inv(0.99999,3);   % 马氏距离
        zeta = sqrt(zeta2);
        
        C=[I,O];      % 位置的均值和协方差矩阵
        ex=C*mean(es(6*(t-1)+1:t*6,:)')';
        X=C*cov(es(6*(t-1)+1:t*6,:)')*C';
        [U,Lam] = eig(X) ;
        
        %  eig估计的特征值精度不够，根据样本协方差矩阵是半正定矩阵（特征值>=0这一特性
        %  把求出的略小于0的特征值取0
        for i = 1:size(Lam)
            for j =1:size(Lam)
                if(Lam(i,j)<0)
                    Lam(i,j)=0;
                end
            end
        end
        %等间隔取点
        lambda = linspace(-pi/2,pi/2,col);
        alpha = linspace(0,2*pi,col);
        %随机取点
        %      lambda =pi*rand([1,col])-pi/2;   %椭球分布栅格
        %      alpha=2*pi*rand([1,row]);
        
        for i=1:col
            for j=1:row
                y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
                x=ex+U*Lam^(1/2)*y;
                x=real(x)/1000;
                a=(i-1)*col+j;
                xx7(a,:)=x;
                %             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','.','color','m','MarkerSize',6);
            end
        end
        %      xx=xx-z_stand(6*(t-1)+1:6*(t-1)+3,1)'/1e3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    xx7=roundn(xx7,-4);
    Index=[];
    q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if xx7(i,:)==xx7(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end
    
    xx7([Index],:)=[];
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
      %初始值设定
    beta =10;
    Delta_v = 44.46;                           %Δv(m/s)
    
    z_0 = 75*1000;                            %(m)初始高度
    psi0 = convang(0,'deg','rad');                                %(°)heading angle
    gamma0 = convang(-1,'deg','rad');        %(°)flight path angle
    v0=7.3*1000;                             %(m/s) relative speed
    
    Point=19;
    col=Point;
    row=Point;
    NN= col*row;
    
    O = zeros(3);
    I = eye(3);
    B=[O;I];
    vt0 = zeros(3,NN);
    
    theta0 = convang(100.2,'deg','rad');      %(°W)经度
    phi0 = convang(40.9,'deg','rad');          %(°N)纬度
    %(kg/m2)弹道系数（仅对一个弹道系数进行研究）
    %地球参量
    omegae = 72.9217*10^(-6);                %自转角速度
    re = 6372.8*1000;                        %(m)地球半径
    ge = 9.81;                               %(m/s^2)地面重力加速度
    
    % 初始加速度方向，随机取点
    % phi =pi*rand([1,col])-(pi/2);   %椭球分布栅格,rand([1,col])表示1行，col列在0到1区间内取随机数
    % theta =2*pi*rand([1,row]);
    
    % 初始加速度方向，等间隔取点
    phi = linspace(-pi/2,pi/2,col);
    theta = linspace(0,2*pi,col);             % 0-180是东经，180-360对应着西经的180-0
    
    V0 = diag([0.0025,0.0025,0.0053]);       %初速度矢量协方差矩阵
    
    N=4000;                                 %最大迭代次数
    %初始化
    x1=zeros(1,N);
    x2=zeros(1,N);
    x3=zeros(1,N);
    v1=zeros(1,N);
    v2=zeros(1,N);
    v3=zeros(1,N);
    
    %平均初速度分量
    x1(1)=0;    % 爆炸前一时刻的位置
    x2(1)=0;
    x3(1)=z_0;
    
    v1(1)=v0*cos(gamma0)*cos(psi0);   % 爆炸前一时刻的速度
    v2(1)=v0*cos(gamma0)*sin(psi0);
    v3(1)=v0*sin(gamma0);
    
    %初始位置矢量和速度矢量
    x0e=[x1(1),x2(1),x3(1)]';
    v0e = [v1(1),v2(1),v3(1)]';
    
    ez=zeros(6*N,NN);    % N迭代次数
    Ez=zeros(6*N,1);    % N迭代次数
    Z=zeros(6*N,6);
    z_stand = zeros(6*N,NN); %存储标称运动状态
    %% 描述标称轨迹
    % 以飞机解体前的状态（3个位置分量和3个速度分量），带入运动方程10-15，求出标称轨迹。
    for n=1:N-1
        % w=[0,0,0]'; %假设风为0
        z_stand(6*(n-1)+1:6*n,1) = [x1(n),x2(n),x3(n),v1(n),v2(n),v3(n)]';
        w=[atmoshwm(20, -157, x3(n),'day',226,'seconds',39600,'version','07'),0]';       %t=0时刻，实施水平风模型
        
        % [T,rho]  =  atmosnrlmsise00(x3(n),20,-157, 2010, 4, 0);%%%大气经验密度模型MSISE-00（质谱仪非相干散射雷达扩展2000），T温度，rho密度
        rhoe = 1.752;                         %(kg/m^3)
        H = 6.7*1000;                              %(km)
        rhoo = rhoe*exp(-z_stand(6*(n-1)+3,1)/H);
        h=1;
        g = ge*(re/(re+x3(n)))^2;     % 逆方重力模型
        vn=[v1(n),v2(n),v3(n)]';      %
        va=vn-w;
        va_norm=norm(va); %求空气速率的模
        %三维运动方程
        [x1(n+1),x2(n+1),x3(n+1),v1(n+1),v2(n+1),v3(n+1)] = ...
            stepfunc(x1(n),x2(n),x3(n),v1(n),v2(n),v3(n),h,w(1),w(2),w(3),rhoo,va_norm,g,beta); %核心公式
        end_num=n;
        if (x3(n+1)<0)
            end_num=n-1;
            break;
        end
        
    end
    z_stand = repmat(z_stand(:,1),[1,NN]);
    
    %% 标称轨迹画图
    v1_r=v1(1:end_num);
    v2_r=v2(1:end_num);
    v3_r=v3(1:end_num);
    x1_r=x1(1:end_num)/1000;  %仿真图坐标轴单位为km
    x2_r=x2(1:end_num)/1000;
    x3_r=x3(1:end_num)/1000;
    
    %标称轨迹坐标，就是仿真图中的轨迹
    %通过推导三维平动运动方程，以破碎前的状态向量为初始状态，以平均弹道系数为基准，得到了一条标称弹道。
    x_nt=[x1_r;x2_r;x3_r];       %文献中（26）   标称轨迹表达式,3×1799
    v_nt=[v1_r;v2_r;v3_r];
    
%     % plot3(x1_r,x2_r,x3_r,'LineWidth',1);       % 在3D空间中绘制画出标称轨迹x_nt
% %     view([55 32]);               % 将方位角设置为55度，将高程设置为32度
%     % set(gca,'YDir','reverse');   % 将Y轴反过来
%     set(gca, 'XLim',[0 500]);   % X轴的数据显示范围
%     set(gca, 'YLim',[-25 5]);
% %     set(gca, 'ZLim',[0 20]);
%     
%     xlabel('x1/km');
%     ylabel('x2/km');
% %     zlabel('x3/km');
%     grid on;
%     % set(gca, 'GridLineStyle', ':');  % 设置为虚线
%     % title('18*18点协方差算法(有风)');
    
    %% 各个方向碎片初始速度(NN个（样本点）轨迹初速度)
    for i = 1:col
        for j = 1:row
            
            vt0(1:3,(i-1)*col+j) = v0e+[Delta_v*cos(phi(i)).*sin(theta(j)),Delta_v*cos(phi(i)).*cos(theta(j)),Delta_v*sin(phi(i))]';   % ok
            sn0 = [x0e',v0e']';  % s*0标称轨迹
            s0 = [x0e',vt0(:,(i-1)*col+j)']';  % 实际状态向量
            
            z0 = s0-sn0;    % 实际 - 理论 = 偏差，研究偏差。实际不确定性，波动，随机过程，通过协方差来研究（核心方法）
            Z0 = [O,O;O,V0];     % 协方差矩阵
            ez(1:6,(i-1)*col+j) = z0;   % NN个样本点求均值的初始条件    !!!!!
            Z(1:6,1:6)=Z0;    % 存储协方差矩阵
        end
    end
    
    %% 求碎片的运动散布
    for tn = 1:end_num
        %         h = 1;
        if tn > 1 %用经向纬向风速随高度变化的变化速度作为偏导
            w1=[atmoshwm(20, -157, z_stand(6*(tn-1)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            w2=[atmoshwm(20, -157, z_stand(6*(tn)+3,1),'day',226,'seconds',39600,'version','07'),0]';
            hw(1) =z_stand(6*(tn)+1,1)-z_stand(6*(tn-1)+1,1);
            hw(2) =z_stand(6*(tn)+2,1)-z_stand(6*(tn-1)+2,1);
            hw(3) =z_stand(6*(tn)+3,1)-z_stand(6*(tn-1)+3,1);
            dw = (w2-w1)./2/hw'; %d高
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        else
            dw = [0.0026 0.0015 0];
            ddw = [dw(1) 0 0;0 dw(2) 0;0 0 dw(3)];
        end
        
        g = ge*(re/(re+x3(n)))^2;     %逆方重力模型
        vn = [v_nt(1,tn),v_nt(2,tn),v_nt(3,tn)]';      %当前标称v(123)
        va = vn - w;
        va_norm = norm(va);  %相对空气速度的大小
        
        rhoe = 1.752;   %(kg/m^3)
        H = 6.7*1000;   %(km)
        rhoo = rhoe*exp(-z_stand(6*(tn-1)+3,1)/H);     %大气密度@该海拔高度（x3）
        drho1 = 0;
        drho2 = 0;     %p对x2求偏导
        drho3 = -rhoe/H*exp(-z_stand(6*(tn-1)+3,1)/H);     %公式（17）,这是p对x3求偏导
        drho = [drho1,drho2 drho3]';
        
        % 时变系数矩阵A(t);
        F = zeros(3);   %3*3的零矩阵，初始化
        G = zeros(3);
        delta = eye(3);
        
        for i = 1:3
            for j = 1:3
                if i == j%添加风速以后，式子略有变化，但对结果影响也不大。
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)'+va_norm*ddw(i,j))-drho(j)*va_norm*va(i));
                else
                    F(i,j) = 1/(2*beta)*(rhoo*(va(i)/va_norm*va'*ddw(j,:)')-drho(j)*va_norm*va(i));
                end
                G(i,j) = -rhoo/(2*beta)*((va(i)*va(j)+(va_norm^2)*delta(i,j))/va_norm);
            end
        end
        Fe = [omegae^2,0,0;0,omegae^2*sin(phi0)^2,-omegae^2*sin(phi0)*cos(phi0);0,-omegae^2*sin(phi0)*cos(phi0),omegae^2*cos(phi0)^2];
        Ge = [0,2*omegae*sin(phi0),-2*omegae*cos(phi0);-2*omegae*sin(phi0),0,0;2*omegae*cos(phi0),0,0];
        A = [O,I;F+Fe,G+Ge];    % 系数矩阵
        
        %数值积分
        [ez(6*tn+1:6*(tn+1),:),Z(6*tn+1:6*(tn+1),:)]=zpartfunc(A,ez,z_stand,Z(6*(tn-1)+1:6*tn,:),h,tn);
    end
    es=(ez(7:end_num*6,:)+z_stand(1:end_num*6-6,:));   % 前一时刻（1）时刻的标称轨迹加上后一时刻(2)时刻的时间演化等于所有状态变量的演化
    %% 协方差传播方法描述椭球轨迹
    for t = [ 350 ]
        hold on ;     %hold on在当前轴上保留图，因此添加到轴上的新图不会删除现有图。
        %     plot3(z_stand(6*(t-1)+1,1)/1000,z_stand(6*(t-1)+2,1)/1000,z_stand(6*(t-1)+3,1)/1000,'LineStyle','none', 'Marker','.','MarkerEdgeColor','red');
        %     text(x_nt(1,t)+10,x_nt(2,t),x_nt(3,t),['t=' num2str(t/60)]);
        
        zeta2 = chi2inv(0.99999,3);   % 马氏距离
        zeta = sqrt(zeta2);
        
        C=[I,O];      % 位置的均值和协方差矩阵
        ex=C*mean(es(6*(t-1)+1:t*6,:)')';
        X=C*cov(es(6*(t-1)+1:t*6,:)')*C';
        [U,Lam] = eig(X) ;
        
        %  eig估计的特征值精度不够，根据样本协方差矩阵是半正定矩阵（特征值>=0这一特性
        %  把求出的略小于0的特征值取0
        for i = 1:size(Lam)
            for j =1:size(Lam)
                if(Lam(i,j)<0)
                    Lam(i,j)=0;
                end
            end
        end
        %等间隔取点
        lambda = linspace(-pi/2,pi/2,col);
        alpha = linspace(0,2*pi,col);
        %随机取点
        %      lambda =pi*rand([1,col])-pi/2;   %椭球分布栅格
        %      alpha=2*pi*rand([1,row]);
        
        for i=1:col
            for j=1:row
                y = [zeta*cos(lambda(j))*cos(alpha(i));zeta*cos(lambda(j))*sin(alpha(i));zeta*sin(lambda(j))];
                x=ex+U*Lam^(1/2)*y;
                x=real(x)/1000;
                a=(i-1)*col+j;
                xx11(a,:)=x;
                %             plot3(x(1),x(2),x(3),'LineStyle','none', 'Marker','.','color','m','MarkerSize',6);
            end
        end
        %      xx=xx-z_stand(6*(t-1)+1:6*(t-1)+3,1)'/1e3;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    area_t = t;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    xx11=roundn(xx11,-4);
    Index=[];
    q=1;
    for i=1:Point^2
        for j=(i+1):Point^2
            if xx11(i,:)==xx11(j,:)
                Index(q)=j;
                q=q+1;
            end
        end
    end
    
    xx11([Index],:)=[];
    
    
 
    %% 二维最后一个时刻凸多边型边界显示 
    
    xx_total=[xx3(:,1),xx3(:,2);xx4(:,1),xx4(:,2);xx6(:,1),xx6(:,2);xx7(:,1),xx7(:,2);xx11(:,1),xx11(:,2)]; 
    figure(1);
    [k_total area3]= convhull(xx_total(:,1),xx_total(:,2));
    hold on;
    xlabel('x/km');
    ylabel('y/km');
    plot(xx_total(k_total,1), xx_total(k_total,2),'b-*','MarkerSize',3,'LineWidth',1.5); % 逆时针连接最外层边界点
    set(gca, 'XLim',[min(xx_total(k_total,1))-80 max(xx_total(k_total,1))+80]); % X轴的数据显示范围
    set(gca, 'YLim',[min(xx_total(k_total,2))-30 max(xx_total(k_total,2))+30]);
    grid on;
    hold on;
    
   
    %%  外扩
    hold on;
    p=[xx_total(k_total,1),xx_total(k_total,2)];
    ii=size(k_total,1);% k的行数
    [newp] = expand(p,ii);
    plot(newp(:,1),newp(:,2),'k-*','MarkerSize',2,'LineWidth',1);
   

   
   %% 扩展10km
   % 这里面k值影响expand函数里减去最后一点
%    hold on;
%    p=[xx(k,1), xx(k,2)]; 
%    [newp2]=expand(p);
%    plot(newp2(:,1),newp2(:,2),'k-','LineWidth',1.5,'HandleVisibility','off');
   set(gca, 'XLim',[min(p(:,1))-80 max(p(:,1))+80]); % X轴的数据显示范围
   set(gca, 'YLim',[min(p(:,2))-20 max(xx_total(:,2))+20]);
   
   fill(newp(:,1),newp(:,2),[0.5 0.5 0.5]);
   fill(xx_total(k_total,1),xx_total(k_total,2),'k');
   %得到扩展区域所有点的坐标
   xVector=100:1:450;
   yVector=-40:1:20;
 
   [x,y]=meshgrid(xVector,yVector); % obtain all coordinates
   
   in=inpolygon(x,y,newp(:,1),newp(:,2));
   inX=x(in);
   inY=y(in);
   suipian=[inX,inY];


end  