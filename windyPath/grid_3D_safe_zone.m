%三维网格地图生成器（自己的）

function [E,E_safe,E3d,E3d_safe]=grid_3D_safe_zone(sizeE,d_grid,h,P0,Pend,n_low)


%网格大小
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%垂直网格向量
z_grid=1:d_grid:z_size;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %随机网格生成，在大块中离散高度
% 
% %创建高度为0-1的随机点
% mean_E=0;
% sigma=1;
% k_sigma=2.2; %编辑这个以改变障碍密度
% E=random('Normal',mean_E,sigma,y_size,x_size);
% sigma_obstacle=k_sigma*sigma;
% E=double(E>sigma_obstacle);
% 
% 
% %给节点周围的方块随机分配高度
% 
% %最小障碍高度
% 
% hh_min=3;
% 
% %初始化临时矩阵以供计算
% EE=E;
% 
% for i=1:x_size
%     for j=1:y_size
%         
%         
%         %随机维度的块边界(max -2:+2)
%         k=i-1-round(random('beta',0.5,0.5)):1:i+1+round(random('beta',0.5,0.5));
%         l=j-1-round(random('beta',0.5,0.5)):1:j+1+round(random('beta',0.5,0.5));        
%         
%         
%         %如果网格内的块边界和节点点值较高，
%         if min(k)>0 && min(l)>0 && max(k)<=x_size && max(l)<=y_size && EE(j,i)==1            
%             
%             
%             %给block分配随机值
%             hh=round(random('Normal',0.75*h,0.5*h));
%             
%             
%             %给出高度的最小值和限制的最大高度
%             if hh<hh_min
%                 hh=hh_min;
%             elseif hh>z_size
%                 hh=z_size;
%             end
%             
%             E(l,k)=hh;           
%             
%         end
%     end
% end
% 
% 
% %在起点和终点附近分配低海拔
% E(P0(1)-n_low:P0(1)+n_low,P0(2)-n_low:P0(2)+n_low)=0;
% E(Pend(1)-n_low:Pend(1)+n_low,Pend(2)-n_low:Pend(2)+n_low)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%创建三维网格

%初始化
E3d=zeros(x_size,y_size,z_size);

%创建三维网格矩阵占用指数(0=空闲，1=障碍)
for i=1:z_size  
	E3d(:,:,i)=E>=z_grid(i);       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%在高海拔区域附近建立安全区

%初始化
E_safe=E;

for i=1:x_size
    for j=1:y_size
        
        %检查的邻居节点
        k=i-1:i+1;
        l=j-1:j+1;
        
        %限制网格内的邻居
        if min(k)<1
            k=i:i+1;
        elseif max(k)>x_size
            k=i-1:i;
        end
        if min(l)<1
            l=j:j+1;
        elseif max(l)>y_size
            l=j-1:j;
        end
        
        %评价矩阵
        E_eval=E(l,k);    
            
        %如果我们在一个自由点，附近有障碍物
        if E(j,i)==0 && max(E_eval(:))>0
            %分配邻居节点的最大值
            E_safe(j,i)=max(E_eval(:));
        end
        
        %如果点升高了，在高度上增加一个安全的步骤
        if E_safe(j,i)>0 && E_safe(j,i)<z_size-1
            E_safe(j,i)=E_safe(j,i)+1;            
        end        
        
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%创建三维安全网格矩阵占用指数(0=自由，0.5=安全，1=障碍)

%初始化
E3d_safe=E3d;

for i=1:x_size
    for j=1:y_size
        for k=1:z_size
            
            %检查的邻居节点
            l=i-1:i+1;
            m=j-1:j+1;
            n=k-1:k+1;
            
            %限制网格内的邻居
            if min(l)<1
                l=i:i+1;
            elseif max(l)>x_size
                l=i-1:i;
            end
            if min(m)<1
                m=j:j+1;
            elseif max(m)>y_size
                m=j-1:j;
            end
            if min(n)<1
                n=k:k+1;
            elseif max(n)>z_size
                n=k-1:k;
            end            
            
            %评价矩阵
            E_eval=E3d(m,l,n);            
            
            %如果我们在一个自由点，附近有障碍物
            if E3d(j,i,k)==0 && max(E_eval(:))==1
                %分配邻居节点的安全值
               E3d_safe(j,i,k)=0.5;
            end

        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%在边界附近建立安全区

E([1 end],:)=z_size;
E(:,[1 end])=z_size;

E_safe([1 end],:)=z_size;
E_safe(:,[1 end])=z_size;

E3d([1 end],:,:)=1;
E3d(:,[1 end],:)=1;
E3d(:,:,[1 end])=1;

E3d_safe([1 end],:,:)=1;
E3d_safe(:,[1 end],:)=1;
E3d_safe(:,:,[1 end])=1;

