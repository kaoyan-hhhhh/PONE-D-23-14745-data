function [obstacle,ellipse_radius,ellipse_midpoint_x,ellipse_midpoint_y] = GetObstacle_t_410_h_21d5()

obstacle=Cov_trajectory_point_t_410_h_21d5();
obstacle(:,2)=obstacle(:,2)+60;
obstacle=round(obstacle);

x_min=min(obstacle(:,1));
x_max=max(obstacle(:,1));
y_min=min(obstacle(:,2));
y_max=max(obstacle(:,2));
rx=abs(x_max-x_min)/2;
ry=abs(y_max-y_min)/2;
%Õ÷‘≤≥§∞Îæ∂
if rx<ry
    ellipse_radius=ry;
else
    ellipse_radius=rx;
end
%Õ÷‘≤÷–µ„
ellipse_midpoint_x=x_min+rx;
ellipse_midpoint_y=y_min+ry;
   
end