%����ڵ��Fֵ�����ڵ��Fֵ���뵽����ĩβ
%openlist:open�б��Žڵ�ͽڵ��Fֵ
%node_f_data:Ҫ����Ľڵ��fֵ
function openlist=Binary_heap_insert(openlist,x,y,z,node_f_data)

%�����ڵ���ӵ�β��
open_size=size(openlist);
row=open_size(1);
last=row+1;
openlist(last,:)=[x,y,z,node_f_data];

%�����ѽṹ
while last>=2
   parent=floor(last/2);
%    if openlist(last,4)<openlist(parent,4)
%        temp=openlist(last,4);
%        openlist(last,4)=openlist(parent,4);
%        openlist(parent,4)=temp;
%    end
%    last=parent;
   if openlist(last,4)>=openlist(parent,4)
       break;
   end
   temp=openlist(last,:);
   openlist(last,:)=openlist(parent,:);
   openlist(parent,:)=temp;
   last=parent;
end

   
end

