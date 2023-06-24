%%
%初始（-230，-15）（10，-10）（250，-5）（490，-10）（730，-15）
theta1=acosd(dot([10-250,-10-(-5)],[250-490,-5-(-10)])/(norm([10-250,-10-(-5)])*norm([250-490,-5-(-10)])));
%%
%before_fly_path
theta2=0;
before_fly_path=[-230,-15;10,-10;120,-40;450,-40;490,-10];
hangshu2=size(before_fly_path,1);
for i=1:hangshu2-2
    theta2=theta2+acosd(dot([before_fly_path(i,1)-before_fly_path(i+1,1),before_fly_path(i,2)-before_fly_path(i+1,2)],[before_fly_path(i+1,1)-before_fly_path(i+2,1),before_fly_path(i+1,2)-before_fly_path(i+2,2)])/(norm([before_fly_path(i,1)-before_fly_path(i+1,1),before_fly_path(i,2)-before_fly_path(i+1,2)])*norm([before_fly_path(i+1,1)-before_fly_path(i+2,1),before_fly_path(i+1,2)-before_fly_path(i+2,2)])));    
end
%%
%无风：A
theta6=0;
nowind_a_path=[-230,-15;10,-10;490,-10];
hangshu6=size(nowind_a_path,1);
for i=1:hangshu6-2
      angle6=acosd(dot([nowind_a_path(i,1)-nowind_a_path(i+1,1),nowind_a_path(i,2)-nowind_a_path(i+1,2)],[nowind_a_path(i+1,1)-nowind_a_path(i+2,1),nowind_a_path(i+1,2)-nowind_a_path(i+2,2)])/(norm([nowind_a_path(i,1)-nowind_a_path(i+1,1),nowind_a_path(i,2)-nowind_a_path(i+1,2)])*norm([nowind_a_path(i+1,1)-nowind_a_path(i+2,1),nowind_a_path(i+1,2)-nowind_a_path(i+2,2)])));    
      theta6=theta6+angle6;
%     theta3=theta3+acosd(dot([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)],[nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])/(norm([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)])*norm([nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])));    
end
distance6=zeros(hangshu6,1);
for i=2:hangshu6 
	distance6(i)=sqrt((nowind_a_path(i,1)-nowind_a_path(i-1,1))^2+(nowind_a_path(i,2)-nowind_a_path(i-1,2))^2);      
end
%%
%无风：Lazy theta/基于风险的Lazy theta
theta3=0;
nowind_lazy_path=[-230,45;10,50;23,50;36,50;49,50;62,50;75,50;88,50;101,50;114,50;127,49;140,49;153,48;166,48;179,48;192,47;205,47;490,50];
hangshu3=size(nowind_lazy_path,1);
for i=1:hangshu3-2
      angle3=acosd(dot([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)],[nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])/(norm([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)])*norm([nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])));    
      theta3=theta3+angle3;
%     theta3=theta3+acosd(dot([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)],[nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])/(norm([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)])*norm([nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])));    
end
distance3=zeros(hangshu3,1);
for i=2:hangshu3 
	distance3(i)=sqrt((nowind_lazy_path(i,1)-nowind_lazy_path(i-1,1))^2+(nowind_lazy_path(i,2)-nowind_lazy_path(i-1,2))^2);      
end
%%
%有风：Lazy theta
theta4=0;
wind_lazy_path=[
-230 45
10	50
23	51
36	52
49	53
62	54
75	55
88	56
101	57
114	58
127	59
140	60
153	60
166	61
179	61
192	61
205	61
484 52
490 50];
hangshu4=size(wind_lazy_path,1);
for i=1:hangshu4-2
      angle4=acosd(dot([wind_lazy_path(i,1)-wind_lazy_path(i+1,1),wind_lazy_path(i,2)-wind_lazy_path(i+1,2)],[wind_lazy_path(i+1,1)-wind_lazy_path(i+2,1),wind_lazy_path(i+1,2)-wind_lazy_path(i+2,2)])/(norm([wind_lazy_path(i,1)-wind_lazy_path(i+1,1),wind_lazy_path(i,2)-wind_lazy_path(i+1,2)])*norm([wind_lazy_path(i+1,1)-wind_lazy_path(i+2,1),wind_lazy_path(i+1,2)-wind_lazy_path(i+2,2)])));    
      theta4=theta4+angle4;
      %     theta4=theta4+acosd(dot([wind_lazy_path(i,1)-wind_lazy_path(i+1,1),wind_lazy_path(i,2)-wind_lazy_path(i+1,2)],[wind_lazy_path(i+1,1)-wind_lazy_path(i+2,1),wind_lazy_path(i+1,2)-wind_lazy_path(i+2,2)])/(norm([wind_lazy_path(i,1)-wind_lazy_path(i+1,1),wind_lazy_path(i,2)-wind_lazy_path(i+1,2)])*norm([wind_lazy_path(i+1,1)-wind_lazy_path(i+2,1),wind_lazy_path(i+1,2)-wind_lazy_path(i+2,2)])));    
end
distance4=zeros(hangshu4,1);
for i=2:hangshu4 
	distance4(i)=sqrt((wind_lazy_path(i,1)-wind_lazy_path(i-1,1))^2+(wind_lazy_path(i,2)-wind_lazy_path(i-1,2))^2);      
end
%%
%有风：基于风险Lazy theta
theta5=0;
wind_lazy_fengxian_path=[
-230 45
10	50
23	49
36	48
49	47
62	46
75	45
88	44
101	43
114	42
127	41
140	40
153	39
166	38
179	37
192	36
205	35
425	45
490	50];
hangshu5=size(wind_lazy_fengxian_path,1);
for i=1:hangshu5-2
      angle5=acosd(dot([wind_lazy_fengxian_path(i,1)-wind_lazy_fengxian_path(i+1,1),wind_lazy_fengxian_path(i,2)-wind_lazy_fengxian_path(i+1,2)],[wind_lazy_fengxian_path(i+1,1)-wind_lazy_fengxian_path(i+2,1),wind_lazy_fengxian_path(i+1,2)-wind_lazy_fengxian_path(i+2,2)])/(norm([wind_lazy_fengxian_path(i,1)-wind_lazy_fengxian_path(i+1,1),wind_lazy_fengxian_path(i,2)-wind_lazy_fengxian_path(i+1,2)])*norm([wind_lazy_fengxian_path(i+1,1)-wind_lazy_fengxian_path(i+2,1),wind_lazy_fengxian_path(i+1,2)-wind_lazy_fengxian_path(i+2,2)])));    
      theta5=theta5+angle5;
%     theta5=theta5+acosd(dot([wind_lazy_fengxian_path(i,1)-wind_lazy_fengxian_path(i+1,1),wind_lazy_fengxian_path(i,2)-wind_lazy_fengxian_path(i+1,2)],[wind_lazy_fengxian_path(i+1,1)-wind_lazy_fengxian_path(i+2,1),wind_lazy_fengxian_path(i+1,2)-wind_lazy_fengxian_path(i+2,2)])/(norm([wind_lazy_fengxian_path(i,1)-wind_lazy_fengxian_path(i+1,1),wind_lazy_fengxian_path(i,2)-wind_lazy_fengxian_path(i+1,2)])*norm([wind_lazy_fengxian_path(i+1,1)-wind_lazy_fengxian_path(i+2,1),wind_lazy_fengxian_path(i+1,2)-wind_lazy_fengxian_path(i+2,2)])));    
end
distance5=zeros(hangshu5,1);
for i=2:hangshu5 
	distance5(i)=sqrt((wind_lazy_fengxian_path(i,1)-wind_lazy_fengxian_path(i-1,1))^2+(wind_lazy_fengxian_path(i,2)-wind_lazy_fengxian_path(i-1,2))^2);      
end
%%
%有风：A
theta7=0;
nowind_a_path=[-230,45;10,50;192,50;211,64;401,64;412,53;484,53;485,52;486,52;487,51;488,51;489,50;490,50];
hangshu6=size(nowind_a_path,1);
for i=1:hangshu6-2
      angle6=acosd(dot([nowind_a_path(i,1)-nowind_a_path(i+1,1),nowind_a_path(i,2)-nowind_a_path(i+1,2)],[nowind_a_path(i+1,1)-nowind_a_path(i+2,1),nowind_a_path(i+1,2)-nowind_a_path(i+2,2)])/(norm([nowind_a_path(i,1)-nowind_a_path(i+1,1),nowind_a_path(i,2)-nowind_a_path(i+1,2)])*norm([nowind_a_path(i+1,1)-nowind_a_path(i+2,1),nowind_a_path(i+1,2)-nowind_a_path(i+2,2)])));    
      theta7=theta7+angle6;
%     theta3=theta3+acosd(dot([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)],[nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])/(norm([nowind_lazy_path(i,1)-nowind_lazy_path(i+1,1),nowind_lazy_path(i,2)-nowind_lazy_path(i+1,2)])*norm([nowind_lazy_path(i+1,1)-nowind_lazy_path(i+2,1),nowind_lazy_path(i+1,2)-nowind_lazy_path(i+2,2)])));    
end
distance6=zeros(hangshu6,1);
for i=2:hangshu6 
	distance6(i)=sqrt((nowind_a_path(i,1)-nowind_a_path(i-1,1))^2+(nowind_a_path(i,2)-nowind_a_path(i-1,2))^2);      
end