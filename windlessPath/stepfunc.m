% 知道输入和输出就ok
% 目的求解位置、速度
% 前提保证初始速度和位置ok
% 三维运动方程描述的即为：标称轨迹
function [x1nn,x2nn,x3nn,v1nn,v2nn,v3nn]=stepfunc(x1n,x2n,x3n,v1n,v2n,v3n,h,w1,w2,w3,rho,va,g,beta)

%parameter
% theta0 = convang(-157,'deg','rad');             %(°W)经度
phi0 = convang(20,'deg','rad');                 %(°N)纬度
re = 6372.8*1000;                                    %(m)地球半径
omegae = 72.9217*10^(-6);                       %自转角速度
omega = [0;omegae*cos(phi0);omegae*sin(phi0)];  %相对角速度
 
% xi=[];

%calculate,3维运动方程
x1nn=v1n*h+x1n;        %数值积分
x2nn=v2n*h+x2n;
x3nn=v3n*h+x3n;
v1nn=(-rho/(2*beta)*va*(v1n-w1)-2*(omega(2)*v3n-omega(3)*v2n)+(omegae^2)*x1n)*h+v1n;
v2nn=(-rho/(2*beta)*va*(v2n-w2)-2*omega(3)*v1n-omega(2)*omega(3)*(re+x3n)+x2n*omega(3)^2)*h+v2n;
v3nn=(-rho/(2*beta)*va*(v3n-w3)-g+2*omega(2)*v1n-omega(2)*omega(3)*x2n+(re+x3n)*omega(2)^2)*h+v3n;

end