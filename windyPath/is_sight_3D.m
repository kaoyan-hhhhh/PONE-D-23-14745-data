%检查3D地图上两个单元格之间是否有自由视线。
%在视线范围内返回1，如果返回0.5
%部分视线，否则0
%From:
%Daniel, Nash - Theta star, any-angle path planning on grids
%Modification:
%Inverted x and y to account for our grid coordinate system将x和y倒转来表示我们的网格坐标系
%Check if the evaluation of E remains within the grid limits检查E的评估是否在网格限制范围内
%Extension to 3D case扩展到3D情况

%Limitation:
%Allows a straight line to pass between diagonally touching blocked cells允许直线通过对角线接触阻塞细胞

function sight=is_sight_3D(xb_bound,yb_bound,mapsize,E3d_safe)
%%
%%第二种写法
%环境矩阵大小
x_size=mapsize(1);
y_size=mapsize(2);
% z_size=mapsize(3);

%重命名
x1_0=xb_bound(1);%当前节点的父节点x坐标
x2=xb_bound(2);%当前节点的邻居节点x坐标
y1_0=yb_bound(1);%当前节点的父节点y坐标
y2=yb_bound(2);%当前节点的邻居节点y坐标
% z1_0=zb_bound(1);%当前节点的父节点z坐标
% z2=zb_bound(2);%当前节点的邻居节点z坐标

%初始化
x1=x1_0;
y1=y1_0;
sight=1;%有视线
f=0;

dx=abs(x2-x1_0);
dy=abs(y2-y1_0);
% dz=abs(z2-z1_0);
sx=sign(x2-x1_0);
sy=sign(y2-y1_0);
% sz=sign(z2-z1_0);
% z1=z1_0;
% if dx>=dy && dx>=dz
%     e1=2*dy-dy;
%     e2=2*dz-dx;
%     for i=1:dx
%         x=x+sx;
%         if e1<0
%             e1=e1+2*dy;
%         else
%             y=y+sy;
%             e1=e1+2*(dy-dx);
%         end
%         if e2<0
%             e2=e2+2*dz;
%         else
%             z=z+sz;
%             e2=e2+2*(dz-dx);
%         end
%     end
% end
        
if dx>=dy 
    while x1~=x2
        f=f+dy;
        if f>=dx && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
                sight=0;%有障碍，无视线
                return
            end
            y1=y1+sy;
            f=f-dx;
        end
        if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
            sight=0;%有障碍，无视线
            return
        end
        if dy==0 && E3d_safe(x1+(sx-1)/2,y1)==1 && E3d_safe(x1+(sx-1)/2,y1-1)==1
            sight=0;%有障碍，无视线
            return
        end
        x1=x1+sx;
    end
else
    while y1~=y2
        f=f+dx;
        if f>=dy && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
                sight=0;%有障碍，无视线
                return
            end
            x1=x1+sx;
            f=f-dy;
        end
        if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
            sight=0;%有障碍，无视线
            return
        end
        if dy==0 && E3d_safe(x1,y1+(sy-1)/2)==1 && E3d_safe(x1-1,y1+(sx-1)/2)==1
            sight=0;%有障碍，无视线
            return
        end
        y1=y1+sy;
    end
end

%%
% zz=floor(z1+(sz-1)/2);
% yy=floor(y1+(sy-1)/2);
% xx=floor(x1+(sx-1)/2);
% if dx>=dy && dx>=dz%dx最大
%     while x1~=x2
%         f=f+dy;
%         if f>=dx && 0<yy && yy<=y_size && 0<xx && xx<=x_size && 0<zz && zz<=z_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%有障碍，无视线
%                 return
%             end
%             y1=y1+sy;
%             z1=z1+sz;
%             f=f-dx;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dy==0 && E3d_safe(xx,y1,zz)==1 && E3d_safe(xx,y1-1,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dz==0 && E3d_safe(xx,yy,z1)==1 && E3d_safe(xx,yy,z1-1)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         x1=x1+sx;        
%     end
% elseif dy>=dx && dy>=dz
%     while y1~=y2
%         f=f+dx;
%         if f>=dy && 0<yy && yy<=y_size && 0<xx && xx<=x_size  && 0<zz && zz<=z_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%有障碍，无视线
%                 return
%             end
%             x1=x1+sx;
%             z1=z1+sz;
%             f=f-dy;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dz==0 && E3d_safe(xx,yy,z1)==1 && E3d_safe(xx,yy,z1-1)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dx==0 && E3d_safe(x1,yy,zz)==1 && E3d_safe(x1-1,yy,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         y1=y1+sy;
%     end
% else
%     while z1~=z2
%         f=f+dz;
%         if f>=dy && 0<yy && yy<=y_size && 0<xx && xx<=x_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%有障碍，无视线
%                 return
%             end
%             x1=x1+sx;
%             y1=y1+sy;
%             f=f-dz;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dx==0 && E3d_safe(x1,yy,zz)==1 && E3d_safe(x1-1,yy,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         if dy==0 && E3d_safe(xx,y1,zz)==1 && E3d_safe(xx,y1-1,zz)==1
%             sight=0;%有障碍，无视线
%             return
%         end
%         z1=z1+sz;
%     end
% end
                


%%
%%第一种写法

% %环境矩阵大小
% x_size=mapsize(1);
% y_size=mapsize(2);
% z_size=mapsize(3);
% 
% %重命名
% x1_0=xb_bound(1);%当前节点的父节点x坐标
% x2=xb_bound(2);%当前节点的邻居节点x坐标
% y1_0=yb_bound(1);%当前节点的父节点y坐标
% y2=yb_bound(2);%当前节点的邻居节点y坐标
% z1_0=zb_bound(1);%当前节点的父节点z坐标
% z2=zb_bound(2);%当前节点的邻居节点z坐标
% 
% 
% %距离
% dx=x2-x1_0;
% dy=y2-y1_0;
% dz=z2-z1_0;
% 
% 
% 
% if dy<0
%     dy=-dy;
%     sy=-1;
% else
%     sy=1;
% end
% 
% if dx<0
%     dx=-dx;
%     sx=-1;
% else
%     sx=1;
% end
% 
% if dz<0
%     dz=-dz;
%     sz=-1;
% else
%     sz=1;
% end
% %高度与水平轨迹的夹角
% %P = atan2(Y,X) 返回 Y 和 X 的四象限反正切 (tan-1)，该值必须为实数。
% %返回以弧度表示的 y/x 的反正切
% gamma=atan2(dz,sqrt(dx^2+dy^2));
% 
% 
% %初始化
% x1=x1_0;
% y1=y1_0;
% sight=1;%有视线
% 
% f=0;
% 
% 
% 
% if dy>=dx
%     while y1~=y2
%         f=f+dx;
%         if f>=dy && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             %floor向下取整
%             z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
% %             if z<=0
% %                 z=1;
% %             elseif z>z_size
% %                 z=z_size;
% %             end
%             
%              if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)>0
%                 if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%              end
%             x1=x1+sx;
%             f=f-dy;
%         end
%         
%         if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             
%             z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
% %             if z<=0
% %                 z=1;
% %             elseif z>z_size
% %                 z=z_size;
% %             end
%             
%             if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)>0
%                 if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%             end
%         end
%         
%         if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 1<x1 && x1<=x_size
%             
%             z_1=floor(z1_0+tan(gamma)*sqrt((x1-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
%             if z_1<=0
%                 z_1=1;
%             elseif z_1>z_size
%                 z_1=z_size;
%             end
%             
%             z_2=floor(z1_0+tan(gamma)*sqrt((x1-1-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
%             if z_2<=0
%                 z_2=1;
%             elseif z_2>z_size
%                 z_2=z_size;
%             end
%             
%             if dx==0 && E3d_safe(x1,y1+(sy-1)/2,z_1)>0 && E3d_safe(x1-1,y1+(sy-1)/2,z_2)>0 
%                 if E3d_safe(x1,y1+(sy-1)/2,z_1)==1 && E3d_safe(x1-1,y1+(sy-1)/2,z_2)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%             end
%         end
%         
%         y1=y1+sy;
%     end
% else
%     while x1~=x2
%         f=f+dy;
%         if f>=dx && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             
%             z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
% %             if z<=0
% %                 z=1;
% %             elseif z>z_size
% %                 z=z_size;
% %             end
%             
%             
%             if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)>0
%                 if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%             end
%             y1=y1+sy;
%             f=f-dx;
%         end
%         if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             
%             z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
% %             if z<=0
% %                 z=1;
% %             elseif z>z_size
% %                 z=z_size;
% %             end
%             
%             if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)>0
%                 if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2,z)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%             end
%         end
%         if 1<y1 && y1<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             
%             z_1=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1-y1_0)^2));
%             if z_1<=0
%                 z_1=1;
%             elseif z_1>z_size
%                 z_1=z_size;
%             end
%             
%             z_2=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1-1-y1_0)^2));
%             if z_2<=0
%                 z_2=1;
%             elseif z_2>z_size
%                 z_2=z_size;
%             end
%             
%             if dy==0 && E3d_safe(x1+(sx-1)/2,y1,z_1)>0 && E3d_safe(x1+(sx-1)/2,y1-1,z_2)>0
%                 if E3d_safe(x1+(sx-1)/2,y1,z_1)==1 && E3d_safe(x1+(sx-1)/2,y1-1,z_2)==1
%                     sight=0;
%                     return
% %                 else
% %                     sight=0.5;
%                     %sight=1;
%                 end
%             end
%         end
%         x1=x1+sx;
%     end
end
