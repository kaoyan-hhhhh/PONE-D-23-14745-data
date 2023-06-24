%��ά�����ͼ���������Լ��ģ�

function [E,E_safe,E3d,E3d_safe]=grid_3D_safe_zone(sizeE,d_grid,h,P0,Pend,n_low)


%�����С
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%��ֱ��������
z_grid=1:d_grid:z_size;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %����������ɣ��ڴ������ɢ�߶�
% 
% %�����߶�Ϊ0-1�������
% mean_E=0;
% sigma=1;
% k_sigma=2.2; %�༭����Ըı��ϰ��ܶ�
% E=random('Normal',mean_E,sigma,y_size,x_size);
% sigma_obstacle=k_sigma*sigma;
% E=double(E>sigma_obstacle);
% 
% 
% %���ڵ���Χ�ķ����������߶�
% 
% %��С�ϰ��߶�
% 
% hh_min=3;
% 
% %��ʼ����ʱ�����Թ�����
% EE=E;
% 
% for i=1:x_size
%     for j=1:y_size
%         
%         
%         %���ά�ȵĿ�߽�(max -2:+2)
%         k=i-1-round(random('beta',0.5,0.5)):1:i+1+round(random('beta',0.5,0.5));
%         l=j-1-round(random('beta',0.5,0.5)):1:j+1+round(random('beta',0.5,0.5));        
%         
%         
%         %��������ڵĿ�߽�ͽڵ��ֵ�ϸߣ�
%         if min(k)>0 && min(l)>0 && max(k)<=x_size && max(l)<=y_size && EE(j,i)==1            
%             
%             
%             %��block�������ֵ
%             hh=round(random('Normal',0.75*h,0.5*h));
%             
%             
%             %�����߶ȵ���Сֵ�����Ƶ����߶�
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
% %�������յ㸽������ͺ���
% E(P0(1)-n_low:P0(1)+n_low,P0(2)-n_low:P0(2)+n_low)=0;
% E(Pend(1)-n_low:Pend(1)+n_low,Pend(2)-n_low:Pend(2)+n_low)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ά����

%��ʼ��
E3d=zeros(x_size,y_size,z_size);

%������ά�������ռ��ָ��(0=���У�1=�ϰ�)
for i=1:z_size  
	E3d(:,:,i)=E>=z_grid(i);       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�ڸߺ������򸽽�������ȫ��

%��ʼ��
E_safe=E;

for i=1:x_size
    for j=1:y_size
        
        %�����ھӽڵ�
        k=i-1:i+1;
        l=j-1:j+1;
        
        %���������ڵ��ھ�
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
        
        %���۾���
        E_eval=E(l,k);    
            
        %���������һ�����ɵ㣬�������ϰ���
        if E(j,i)==0 && max(E_eval(:))>0
            %�����ھӽڵ�����ֵ
            E_safe(j,i)=max(E_eval(:));
        end
        
        %����������ˣ��ڸ߶�������һ����ȫ�Ĳ���
        if E_safe(j,i)>0 && E_safe(j,i)<z_size-1
            E_safe(j,i)=E_safe(j,i)+1;            
        end        
        
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ά��ȫ�������ռ��ָ��(0=���ɣ�0.5=��ȫ��1=�ϰ�)

%��ʼ��
E3d_safe=E3d;

for i=1:x_size
    for j=1:y_size
        for k=1:z_size
            
            %�����ھӽڵ�
            l=i-1:i+1;
            m=j-1:j+1;
            n=k-1:k+1;
            
            %���������ڵ��ھ�
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
            
            %���۾���
            E_eval=E3d(m,l,n);            
            
            %���������һ�����ɵ㣬�������ϰ���
            if E3d(j,i,k)==0 && max(E_eval(:))==1
                %�����ھӽڵ�İ�ȫֵ
               E3d_safe(j,i,k)=0.5;
            end

        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�ڱ߽總��������ȫ��

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

