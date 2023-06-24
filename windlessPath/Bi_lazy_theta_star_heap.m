%lazy theta�㷨����
function [ path,route_node_number,path_backwards1,path_backwards2 ] = Bi_lazy_theta_star_heap(xstart,ystart,zstart,xend,yend,zend,mapsize,obstacle,E3d_safe)

%��������(��ͼ)�Ĵ�С��x y z�ᣩ
x_size=mapsize(1);
y_size=mapsize(2);
z_size=mapsize(3);

%���嵽���ھӵĽڵ�
%%��mapsizeΪ5*5*5�ľ�����zeros(mapsize)������5*5*5��ȫ�����
%����
came_fromx1=zeros(mapsize);
came_fromy1=zeros(mapsize);
came_fromz1=zeros(mapsize);
came_fromx1(xstart,ystart,zstart)=xstart;
came_fromy1(xstart,ystart,zstart)=ystart;
came_fromz1(xstart,ystart,zstart)=zstart;
%����
came_fromx2=zeros(mapsize);
came_fromy2=zeros(mapsize);
came_fromz2=zeros(mapsize);
came_fromx2(xend,yend,zend)=xend;
came_fromy2(xend,yend,zend)=yend;
came_fromz2(xend,yend,zend)=zend;

%����openlist����Žڵ�λ����Ϣ
%open=[xstart,ystart,zstart];
%����closelist����Žڵ�λ����Ϣ
%����
close1=[];
%����
close2=[];
%��������G��Fֵ��G:��㵽��ǰ�ڵ�ľ��롣F:��ǰ�ڵ㵽�յ�ľ��롣
%Inf*ones������mapsizeά����ͬ��һ������,�����ÿ��ֵ���������
G1=Inf*ones(mapsize);
G1(xstart,ystart,zstart)=0;
F1=Inf*ones(mapsize);
F1(xstart,ystart,zstart)=sqrt((xend-xstart)^2+(yend-ystart)^2+(zend-zstart)^2);

%����
G2=Inf*ones(mapsize);
G2(xend,yend,zend)=0;
F2=Inf*ones(mapsize);
F2(xend,yend,zend)=sqrt((xstart-xend)^2+(ystart-yend)^2+(zstart-zend)^2);

% g(sstart)=0;
% parent(sstart)=sstart; 
% open.insert(start,g(sstart)+h(start));
%����
open1=[xstart,ystart,zstart,F1(xstart,ystart,zstart)];
%����
open2=[xend,yend,zend,F2(xend,yend,zend)];

%��ʼ��
find_path=0;
gf=2;%��������

%��openlist�б�Ϊ��ʱ
while isempty(open1)==0 && isempty(open2)==0 && find_path==0
%����openlist��fx��С�Ľڵ�
%����openlist��ÿһ���ڵ��fֵ
%����fxlist��openlist��ÿһ���ڵ��fֵ������Сֵ������
%��openlist��ȡ��fֵ��С�Ľڵ㣬��ֵ��cnode
   
     [xcurrent1, ycurrent1, zcurrent1]=Binary_heap_pop(open1);
     current_node1=[xcurrent1, ycurrent1, zcurrent1];

     %�������������㣨����Ϊ[��ǰ�ڵ�ĸ��ڵ�  ��ǰ�ڵ�]��
     xb1=[came_fromx1(xcurrent1,ycurrent1,zcurrent1) xcurrent1];
     yb1=[came_fromy1(xcurrent1,ycurrent1,zcurrent1) ycurrent1];
     zb1=[came_fromz1(xcurrent1,ycurrent1,zcurrent1) zcurrent1];

     %�����Ľڵ�(��ĸ)���ھ�֮������߼��
     sight=is_sight_3D(xb1,yb1,zb1,mapsize,E3d_safe); 

     %���û������
     if sight==0 

         %��ʼ����Сgֵ��InfΪ�����
         g_min=Inf;

         %�����ھӽڵ�
         for i=-1:1
             for j=-1:1
                 for k=-1:1 
                     %������ڽڵ���������
                     if xcurrent1+i>0 && ycurrent1+j>0 && zcurrent1+k>0 && xcurrent1+i<=x_size && ycurrent1+j<=y_size && zcurrent1+k<=z_size

                         %����ھӽڵ��ڹر��б���
                         if max(sum([xcurrent1+i==close1(:,1) ycurrent1+j==close1(:,2) zcurrent1+k==close1(:,3)],2))==3

                             %����ӿ�ʼ���ھӽڵ�+���ھӽڵ㵽��ǰ�ڵ�ľ���
                             g_test=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)+sqrt(i^2+j^2+k^2);                            

                             %��������������С�ģ���������ָ����ǰ�ڵ���ھ�Ϊ��ǰ�ڵ�ĸ��ڵ�
                             if g_test<g_min

                                 g_min=g_test;  
                                 G1(xcurrent1,ycurrent1,zcurrent1)=g_test;

                                 came_fromx1(xcurrent1,ycurrent1,zcurrent1)=xcurrent1+i;
                                 came_fromy1(xcurrent1,ycurrent1,zcurrent1)=ycurrent1+j;
                                 came_fromz1(xcurrent1,ycurrent1,zcurrent1)=zcurrent1+k;
                             end
                         end
                     end
                 end
             end
         end
     end
   
   
   

     [xcurrent2, ycurrent2, zcurrent2]=Binary_heap_pop(open2);
     current_node2=[xcurrent2, ycurrent2, zcurrent2];

     %�������������㣨����Ϊ[��ǰ�ڵ�ĸ��ڵ�  ��ǰ�ڵ�]��
     xb2=[came_fromx2(xcurrent2,ycurrent2,zcurrent2) xcurrent2];
     yb2=[came_fromy2(xcurrent2,ycurrent2,zcurrent2) ycurrent2];
     zb2=[came_fromz2(xcurrent2,ycurrent2,zcurrent2) zcurrent2];

     %�����Ľڵ�(��ĸ)���ھ�֮������߼��
     sight=is_sight_3D(xb2,yb2,zb2,mapsize,E3d_safe); 

     %���û������
     if sight==0 

         %��ʼ����Сgֵ��InfΪ�����
         g_min=Inf;

         %�����ھӽڵ�
         for i=-1:1
             for j=-1:1
                 for k=-1:1 
                     %������ڽڵ���������
                     if xcurrent2+i>0 && ycurrent2+j>0 && zcurrent2+k>0 && xcurrent2+i<=x_size && ycurrent2+j<=y_size && zcurrent2+k<=z_size

                         %����ھӽڵ��ڹر��б���
                         if max(sum([xcurrent2+i==close2(:,1) ycurrent2+j==close2(:,2) zcurrent2+k==close2(:,3)],2))==3

                             %����ӿ�ʼ���ھӽڵ�+���ھӽڵ㵽��ǰ�ڵ�ľ���
                             g_test=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)+sqrt(i^2+j^2+k^2);                            

                             %��������������С�ģ���������ָ����ǰ�ڵ���ھ�Ϊ��ǰ�ڵ�ĸ��ڵ�
                             if g_test<g_min

                                 g_min=g_test;  
                                 G2(xcurrent2,ycurrent2,zcurrent2)=g_test;

                                 came_fromx1(xcurrent2,ycurrent2,zcurrent2)=xcurrent2+i;
                                 came_fromy1(xcurrent2,ycurrent2,zcurrent2)=ycurrent2+j;
                                 came_fromz1(xcurrent2,ycurrent2,zcurrent2)=zcurrent2+k;
                             end
                         end
                     end
                 end
             end
         end
     end
   
     
     

     %����Ƿ񵽴�Ŀ��ڵ�
     if xcurrent1==xcurrent2 && ycurrent1==ycurrent2 && zcurrent1==zcurrent2
           %����Ŀ��ڵ㣬�ҵ������·�����˳�������·��
           find_path=1;
        
     end
           %��openlist��ȡ��fֵ��С�Ľڵ�cnode���뵽���ر��б�closelist����
           close1(size(close1,1)+1,:)=current_node1;
           %��openlist��fֵ��С�Ľڵ�ӡ������б�openlist����ɾ��
           open1=Binary_heap_remove(open1);
           
           %��openlist��ȡ��fֵ��С�Ľڵ�cnode���뵽���ر��б�closelist����
           close2(size(close2,1)+1,:)=current_node2;
           %��openlist��fֵ��С�Ľڵ�ӡ������б�openlist����ɾ��
           open2=Binary_heap_remove(open2);
           
           % ��չ��ǰfx��С�Ľڵ㣬��������һ��ѭ������
           % �����ھӽڵ�
           for i=-1:1
               for j=-1:1
                   for k=-1:1
                         %������ڽڵ���������
                         if xcurrent1+i>0 && ycurrent1+j>0 && zcurrent1+k>0 && xcurrent1+i<=x_size && ycurrent1+j<=y_size && zcurrent1+k<=z_size


                             %����ھӽڵ�Ȳ�����openҲ������close
                             %%��� A �Ǿ����� sum(A) �����ذ���ÿ���ܺ͵���������
                             %%a(:,1)Ϊȡa�����е�һ��Ԫ�أ�a(1,:)Ϊȡa�����е�һ��Ԫ�ء�
                             %%S = sum(A,dim) ��ά�� dim �����ܺ͡����磬��� A Ϊ������ sum(A,2) �ǰ���ÿһ���ܺ͵���������
                             %%xcurrent+j==open(:,1)���� ��open��������*1 ����
                             %check_open:����ھӽڵ��Ƿ���open�б��У�����3��˵����open�б��С�����С��3��˵������open�б��С�
                             %check_closed������ھӽڵ��Ƿ���close�б��У�����3��˵����close�б��С�����С��3��˵������close�б��С�
                             check_open1=max(sum([xcurrent1+i==open1(:,1) ycurrent1+j==open1(:,2) zcurrent1+k==open1(:,3)],2));
                             check_close1=max(sum([xcurrent1+i==close1(:,1) ycurrent1+j==close1(:,2) zcurrent1+k==close1(:,3)],2));
                             check_obstacle=max(sum([xcurrent1+i==obstacle(:,1) ycurrent1+j==obstacle(:,2) zcurrent1+k==obstacle(:,3)],2));

                             %isemptyΪ��ʱ������1��isemptyΪ�ǿ�ʱ������0��
                             if isempty(check_open1)==1
                                 check_open1=0;
                             end

                             if isempty(check_close1)==1
                                 check_close1=0;
                             end

                             if isempty(check_obstacle)==1
                                 check_obstacle=0;
                             end

                             % �ж��ھӽڵ�node�Ƿ���closelist��blocklist��lowrisklist��
                             if check_close1==3 || check_obstacle==3
                                 continue
                             else
                                 % ��if����жϵó����ڵ�node�Ȳ���closelist�У�Ҳ����blocklist��
                                 % ����ھӽڵ�node����openlist��
                                 if check_open1<3
%                                      %��open������ھӽڵ�node
%                                      %size(open,1)����open������
%                                      open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      %����ӿ�ʼ����ǰ�ڵ�ľ���+��ǰ�ڵ㵽�ھӽڵ�ľ���
%                                      g_try=G(xcurrent,ycurrent,zcurrent)+sqrt(i^2+j^2+k^2);
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
%                                      %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
%                                      came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=xcurrent; 
%                                      came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=ycurrent;
%                                      came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;
                                     G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=Inf;
                                     came_fromx1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0; 
                                     came_fromy1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0;
                                     came_fromz1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=0;

                                 end
                                 G_old1=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k);
                                 
                                 % ������жϵó����ڵ�node����closelist�У�����blocklist�У���openlist��

                                 % �������㵽��ǰ�ڵ�ĸ��ڵ�+�ӵ�ǰ�ڵ�ĸ��ڵ㵽��ǰ�ڵ���ھӵľ���
                                 g_try1=G1(came_fromx1(xcurrent1,ycurrent1,zcurrent1),came_fromy1(xcurrent1,ycurrent1,zcurrent1),came_fromz1(xcurrent1,ycurrent1,zcurrent1))+sqrt((came_fromx1(xcurrent1,ycurrent1,zcurrent1)-(xcurrent1+i))^2+(came_fromy1(xcurrent1,ycurrent1,zcurrent1)-(ycurrent1+j))^2+(came_fromz1(xcurrent1,ycurrent1,zcurrent1)-(zcurrent1+k))^2);

                                 % ����������С���ھӵľ���
                                 if g_try1<G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)

                                     %����ȷ��·�ϣ�������Ϣ

                                     %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
                                     came_fromx1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromx1(xcurrent1,ycurrent1,zcurrent1); 
                                     came_fromy1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromy1(xcurrent1,ycurrent1,zcurrent1);
                                     came_fromz1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=came_fromz1(xcurrent1,ycurrent1,zcurrent1);

                                     %�����ɱ�����
                                     G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=g_try1;
                                     H1=sqrt((xend-(xcurrent1+i))^2+(yend-(ycurrent1+j))^2+(zend-(zcurrent1+k))^2);
                                     %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                     F1(xcurrent1+i,ycurrent1+j,zcurrent1+k)=G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)+gf*H1;
                                 end
                                 if  G_old1>G1(xcurrent1+i,ycurrent1+j,zcurrent1+k)
                                      if check_open1==3
                                          open1=Binary_heap_remove(open1);
                                      end
                                          open1=Binary_heap_insert(open1,xcurrent1+i,ycurrent1+j,zcurrent1+k,F1(xcurrent1+i,ycurrent1+j,zcurrent1+k));
                                     

%                                      if check_open<3
%                                             open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      end
%                                      %�����ɱ�����
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
                                      
                                    
                                 end
                                
                             end
                      


                         end
                   end
               end
           end
           
           % ��չ��ǰfx��С�Ľڵ㣬��������һ��ѭ������
           % �����ھӽڵ�
           for i=-1:1
               for j=-1:1
                   for k=-1:1
                         %������ڽڵ���������
                         if xcurrent2+i>0 && ycurrent2+j>0 && zcurrent2+k>0 && xcurrent2+i<=x_size && ycurrent2+j<=y_size && zcurrent2+k<=z_size


                             %����ھӽڵ�Ȳ�����openҲ������close
                             %%��� A �Ǿ����� sum(A) �����ذ���ÿ���ܺ͵���������
                             %%a(:,1)Ϊȡa�����е�һ��Ԫ�أ�a(1,:)Ϊȡa�����е�һ��Ԫ�ء�
                             %%S = sum(A,dim) ��ά�� dim �����ܺ͡����磬��� A Ϊ������ sum(A,2) �ǰ���ÿһ���ܺ͵���������
                             %%xcurrent+j==open(:,1)���� ��open��������*1 ����
                             %check_open:����ھӽڵ��Ƿ���open�б��У�����3��˵����open�б��С�����С��3��˵������open�б��С�
                             %check_closed������ھӽڵ��Ƿ���close�б��У�����3��˵����close�б��С�����С��3��˵������close�б��С�
                             check_open2=max(sum([xcurrent2+i==open2(:,1) ycurrent2+j==open2(:,2) zcurrent2+k==open2(:,3)],2));
                             check_close2=max(sum([xcurrent2+i==close2(:,1) ycurrent2+j==close2(:,2) zcurrent2+k==close2(:,3)],2));
                             check_obstacle=max(sum([xcurrent2+i==obstacle(:,1) ycurrent2+j==obstacle(:,2) zcurrent2+k==obstacle(:,3)],2));

                             %isemptyΪ��ʱ������1��isemptyΪ�ǿ�ʱ������0��
                             if isempty(check_open2)==1
                                 check_open2=0;
                             end

                             if isempty(check_close2)==1
                                 check_close2=0;
                             end

                             if isempty(check_obstacle)==1
                                 check_obstacle=0;
                             end

                             % �ж��ھӽڵ�node�Ƿ���closelist��blocklist��lowrisklist��
                             if check_close2==3 || check_obstacle==3
                                 continue
                             else
                                 % ��if����жϵó����ڵ�node�Ȳ���closelist�У�Ҳ����blocklist��
                                 % ����ھӽڵ�node����openlist��
                                 if check_open2<3
%                                      %��open������ھӽڵ�node
%                                      %size(open,1)����open������
%                                      open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      %����ӿ�ʼ����ǰ�ڵ�ľ���+��ǰ�ڵ㵽�ھӽڵ�ľ���
%                                      g_try=G(xcurrent,ycurrent,zcurrent)+sqrt(i^2+j^2+k^2);
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
%                                      %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
%                                      came_fromx(xcurrent+i,ycurrent+j,zcurrent+k)=xcurrent; 
%                                      came_fromy(xcurrent+i,ycurrent+j,zcurrent+k)=ycurrent;
%                                      came_fromz(xcurrent+i,ycurrent+j,zcurrent+k)=zcurrent;
                                     G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=Inf;
                                     came_fromx2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0; 
                                     came_fromy2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0;
                                     came_fromz2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=0;

                                 end
                                 G_old=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k);
                                 
                                 % ������жϵó����ڵ�node����closelist�У�����blocklist�У���openlist��

                                 % �������㵽��ǰ�ڵ�ĸ��ڵ�+�ӵ�ǰ�ڵ�ĸ��ڵ㵽��ǰ�ڵ���ھӵľ���
                                 g_try=G2(came_fromx2(xcurrent2,ycurrent2,zcurrent2),came_fromy2(xcurrent2,ycurrent2,zcurrent2),came_fromz2(xcurrent2,ycurrent2,zcurrent2))+sqrt((came_fromx2(xcurrent2,ycurrent2,zcurrent2)-(xcurrent2+i))^2+(came_fromy2(xcurrent1,ycurrent1,zcurrent1)-(ycurrent1+j))^2+(came_fromz2(xcurrent1,ycurrent1,zcurrent1)-(zcurrent1+k))^2);

                                 % ����������С���ھӵľ���
                                 if g_try<G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)

                                     %����ȷ��·�ϣ�������Ϣ

                                     %��¼���ĸ��ڵ㵽���ھ�,��Ÿ��ڵ�
                                     came_fromx2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromx2(xcurrent2,ycurrent2,zcurrent2); 
                                     came_fromy2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromy2(xcurrent2,ycurrent2,zcurrent2);
                                     came_fromz2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=came_fromz2(xcurrent2,ycurrent2,zcurrent2);

                                     %�����ɱ�����
                                     G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=g_try;
                                     H2=sqrt((xend-(xcurrent2+i))^2+(yend-(ycurrent2+j))^2+(zend-(zcurrent2+k))^2);
                                     %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                     F2(xcurrent2+i,ycurrent2+j,zcurrent2+k)=G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)+gf*H2;
                                 end
                                 if  G_old>G2(xcurrent2+i,ycurrent2+j,zcurrent2+k)
                                      if check_open2==3
                                          open2=Binary_heap_remove(open2);
                                      end
                                          open2=Binary_heap_insert(open2,xcurrent2+i,ycurrent2+j,zcurrent2+k,F2(xcurrent2+i,ycurrent2+j,zcurrent2+k));
                                     

%                                      if check_open<3
%                                             open(size(open,1)+1,:)=[xcurrent+i, ycurrent+j, zcurrent+k];
%                                      end
%                                      %�����ɱ�����
%                                      G(xcurrent+i,ycurrent+j,zcurrent+k)=g_try;
%                                      H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
%                                      %F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
%                                      F(xcurrent+i,ycurrent+j,zcurrent+k)=G(xcurrent+i,ycurrent+j,zcurrent+k)+gf*H;
                                      
                                    
                                 end
                                
                             end
                      


                         end
                   end
               end
           end
           
           
           
           
           
           
           
           
end


             
             
%�����ؽ�·����֪�����ĸ��ڵ㵽����ھ�


%��һ��Ԫ���ǵ����
path_backwards1=[xcurrent1,ycurrent1,zcurrent1];

%��ʼ��
i=2;

%����㻹û�е���
while xcurrent1~=xstart || ycurrent1~=ystart || zcurrent1~=zstart
    
    path_backwards1(i,:)=[came_fromx1(xcurrent1,ycurrent1,zcurrent1) came_fromy1(xcurrent1,ycurrent1,zcurrent1) came_fromz1(xcurrent1,ycurrent1,zcurrent1)];
    xcurrent1=path_backwards1(i,1);
    ycurrent1=path_backwards1(i,2);    
    zcurrent1=path_backwards1(i,3); 
    i=i+1;
    
end

%·���ڵ�����
route_node_number1=size(path_backwards1,1);


%����·������
% path=path_backwards(route_node_number+1-(1:route_node_number),:);

path_backwards2=[xcurrent2,ycurrent2,zcurrent2];

%��ʼ��
i=2;

%����㻹û�е���
while xcurrent2~=xend || ycurrent2~=yend || zcurrent2~=zend
    
    path_backwards2(i,:)=[came_fromx2(xcurrent2,ycurrent2,zcurrent2) came_fromy2(xcurrent2,ycurrent2,zcurrent2) came_fromz2(xcurrent2,ycurrent2,zcurrent2)];
    xcurrent2=path_backwards2(i,1);
    ycurrent2=path_backwards2(i,2);    
    zcurrent2=path_backwards2(i,3); 
    i=i+1;
    
end

%·���ڵ�����
route_node_number2=size(path_backwards2,1);
route_node_number = route_node_number1 + route_node_number2 - 1;

for i=1:route_node_number1-1
         path_backwards(i,:)=path_backwards1(route_node_number1+1-i,:);
end

path=[path_backwards;path_backwards2];
% path=path_backwards1(route_node_number+1-(1:route_node_number),:);


%���·����ʼ�Ŀ�ʼ�ͽ�����
path(1,:)=[xstart ystart zstart];
path(route_node_number,:)=[xend yend zend];
    

end
     
 

        

