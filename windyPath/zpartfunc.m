% 迭代，1024个初始时刻，开始迭代
function [eznn,Znn]=zpartfunc(An,ez,z_stand,Zn,h,tn)
        B=[zeros(3,3);eye(3,3)];
% eznn = (An *  ezn) * h + ezn;     %公式36
% Znn = (An * Zn + Zn * An') * h + Zn;    %公式37
        ezn=ez(6*(tn-1)+1:6*tn,:);        %***根据公式38
        eznn=An*ezn*h+ezn;    
        Zn2 = cov(ezn');                  %***根据公式39求协方差
        Znn = (An * Zn2 + Zn2 * An')*h+Zn;

end
