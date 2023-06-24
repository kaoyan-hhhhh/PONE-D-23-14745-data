function suipian = GetSuiPian()

suipian =Cov_trajectory_point();
suipian(:,2)=suipian(:,2)+60;

x_min=min(suipian(:,1));
x_max=max(suipian(:,1));
y_min=min(suipian(:,2));
y_max=max(suipian(:,2));
z_min=min(suipian(:,3));
z_max=max(suipian(:,3));
end