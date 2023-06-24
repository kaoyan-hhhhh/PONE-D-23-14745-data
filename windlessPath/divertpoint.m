%% 已知两条直线的斜率和截距，求交点坐标
function [a,b,c,d,y_1,y_2,y_3]=divertpoint(a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,e_x,e_y)
%     P = [a_x a_y; b_x b_y; c_x c_y;d_x d_y;e_x  e_y];
%     plot(P(:,1),P(:,2),'b*');
    hold on;
    x=-200:0.1:500;

    %平行线
    k_1=-(d_y-b_y)/(b_x-d_x);
    b_1=-(c_x*b_y+d_x*c_y-b_x*c_y-c_x* d_y)/(b_x-d_x);
    y_1=k_1*x+b_1;
%     plot(x,y_1,'r');
    hold on;

    %第一条斜线A B
    k_2=(b_y-a_y)/(b_x-a_x);
    b_2=a_y-k_2*a_x;
    y_2=k_2*x+b_2;
%     plot(x,y_2,'g');
    hold on;

    %第一条斜线D E
    k_3=(e_y-d_y)/(e_x-d_x);
    b_3=d_y-k_3*d_x;
    y_3=k_3*x+b_3;
%     plot(x,y_3,'b');

    [a,b]=linecross(k_1,b_1,k_2,b_2);
    [c,d]=linecross(k_1,b_1,k_3,b_3);

  end
