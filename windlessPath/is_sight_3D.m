%���3D��ͼ��������Ԫ��֮���Ƿ����������ߡ�
%�����߷�Χ�ڷ���1���������0.5
%�������ߣ�����0
%From:
%Daniel, Nash - Theta star, any-angle path planning on grids
%Modification:
%Inverted x and y to account for our grid coordinate system��x��y��ת����ʾ���ǵ���������ϵ
%Check if the evaluation of E remains within the grid limits���E�������Ƿ����������Ʒ�Χ��
%Extension to 3D case��չ��3D���

%Limitation:
%Allows a straight line to pass between diagonally touching blocked cells����ֱ��ͨ���Խ��߽Ӵ�����ϸ��

function sight=is_sight_3D(xb_bound,yb_bound,mapsize,E3d_safe)
%%
%%�ڶ���д��
%���������С
x_size=mapsize(1);
y_size=mapsize(2);
% z_size=mapsize(3);

%������
x1_0=xb_bound(1);%��ǰ�ڵ�ĸ��ڵ�x����
x2=xb_bound(2);%��ǰ�ڵ���ھӽڵ�x����
y1_0=yb_bound(1);%��ǰ�ڵ�ĸ��ڵ�y����
y2=yb_bound(2);%��ǰ�ڵ���ھӽڵ�y����
% z1_0=zb_bound(1);%��ǰ�ڵ�ĸ��ڵ�z����
% z2=zb_bound(2);%��ǰ�ڵ���ھӽڵ�z����

%��ʼ��
x1=x1_0;
y1=y1_0;
sight=1;%������
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
                sight=0;%���ϰ���������
                return
            end
            y1=y1+sy;
            f=f-dx;
        end
        if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
            sight=0;%���ϰ���������
            return
        end
        if dy==0 && E3d_safe(x1+(sx-1)/2,y1)==1 && E3d_safe(x1+(sx-1)/2,y1-1)==1
            sight=0;%���ϰ���������
            return
        end
        x1=x1+sx;
    end
else
    while y1~=y2
        f=f+dx;
        if f>=dy && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            if E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
                sight=0;%���ϰ���������
                return
            end
            x1=x1+sx;
            f=f-dy;
        end
        if f~=0 && E3d_safe(x1+(sx-1)/2,y1+(sy-1)/2)==1
            sight=0;%���ϰ���������
            return
        end
        if dy==0 && E3d_safe(x1,y1+(sy-1)/2)==1 && E3d_safe(x1-1,y1+(sx-1)/2)==1
            sight=0;%���ϰ���������
            return
        end
        y1=y1+sy;
    end
end

%%
% zz=floor(z1+(sz-1)/2);
% yy=floor(y1+(sy-1)/2);
% xx=floor(x1+(sx-1)/2);
% if dx>=dy && dx>=dz%dx���
%     while x1~=x2
%         f=f+dy;
%         if f>=dx && 0<yy && yy<=y_size && 0<xx && xx<=x_size && 0<zz && zz<=z_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%���ϰ���������
%                 return
%             end
%             y1=y1+sy;
%             z1=z1+sz;
%             f=f-dx;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dy==0 && E3d_safe(xx,y1,zz)==1 && E3d_safe(xx,y1-1,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dz==0 && E3d_safe(xx,yy,z1)==1 && E3d_safe(xx,yy,z1-1)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         x1=x1+sx;        
%     end
% elseif dy>=dx && dy>=dz
%     while y1~=y2
%         f=f+dx;
%         if f>=dy && 0<yy && yy<=y_size && 0<xx && xx<=x_size  && 0<zz && zz<=z_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%���ϰ���������
%                 return
%             end
%             x1=x1+sx;
%             z1=z1+sz;
%             f=f-dy;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dz==0 && E3d_safe(xx,yy,z1)==1 && E3d_safe(xx,yy,z1-1)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dx==0 && E3d_safe(x1,yy,zz)==1 && E3d_safe(x1-1,yy,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         y1=y1+sy;
%     end
% else
%     while z1~=z2
%         f=f+dz;
%         if f>=dy && 0<yy && yy<=y_size && 0<xx && xx<=x_size
%             if E3d_safe(xx,yy,zz)==1
%                 sight=0;%���ϰ���������
%                 return
%             end
%             x1=x1+sx;
%             y1=y1+sy;
%             f=f-dz;
%         end
%         if f~=0 && E3d_safe(xx,yy,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dx==0 && E3d_safe(x1,yy,zz)==1 && E3d_safe(x1-1,yy,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         if dy==0 && E3d_safe(xx,y1,zz)==1 && E3d_safe(xx,y1-1,zz)==1
%             sight=0;%���ϰ���������
%             return
%         end
%         z1=z1+sz;
%     end
% end
                


%%
%%��һ��д��

% %���������С
% x_size=mapsize(1);
% y_size=mapsize(2);
% z_size=mapsize(3);
% 
% %������
% x1_0=xb_bound(1);%��ǰ�ڵ�ĸ��ڵ�x����
% x2=xb_bound(2);%��ǰ�ڵ���ھӽڵ�x����
% y1_0=yb_bound(1);%��ǰ�ڵ�ĸ��ڵ�y����
% y2=yb_bound(2);%��ǰ�ڵ���ھӽڵ�y����
% z1_0=zb_bound(1);%��ǰ�ڵ�ĸ��ڵ�z����
% z2=zb_bound(2);%��ǰ�ڵ���ھӽڵ�z����
% 
% 
% %����
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
% %�߶���ˮƽ�켣�ļн�
% %P = atan2(Y,X) ���� Y �� X �������޷����� (tan-1)����ֵ����Ϊʵ����
% %�����Ի��ȱ�ʾ�� y/x �ķ�����
% gamma=atan2(dz,sqrt(dx^2+dy^2));
% 
% 
% %��ʼ��
% x1=x1_0;
% y1=y1_0;
% sight=1;%������
% 
% f=0;
% 
% 
% 
% if dy>=dx
%     while y1~=y2
%         f=f+dx;
%         if f>=dy && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
%             %floor����ȡ��
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
