% ������1024����ʼʱ�̣���ʼ����
function [eznn,Znn]=zpartfunc(An,ez,z_stand,Zn,h,tn)
        B=[zeros(3,3);eye(3,3)];
% eznn = (An *  ezn) * h + ezn;     %��ʽ36
% Znn = (An * Zn + Zn * An') * h + Zn;    %��ʽ37
        ezn=ez(6*(tn-1)+1:6*tn,:);        %***���ݹ�ʽ38
        eznn=An*ezn*h+ezn;    
        Zn2 = cov(ezn');                  %***���ݹ�ʽ39��Э����
        Znn = (An * Zn2 + Zn2 * An')*h+Zn;

end
