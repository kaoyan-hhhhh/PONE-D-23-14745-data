%����ڵ��Fֵ�����ڵ��Fֵ���뵽����ĩβ
%openlist:open�б��Žڵ�ͽڵ��Fֵ
%node_coordinate:Ҫ����Ľڵ�����
%node_f_data:Ҫ����Ľڵ��fֵ
function openlist=queue_insert(openlist,x,y,z,node_f_data)

%�����ڵ���ӵ�β��
open_size=size(openlist);
row=open_size(1);
last=row+1;
openlist(last,:)=[x,y,z,node_f_data];
%��ȡopen����
length=last;
%ð�����򣬴�С����
for i=1:length-1
    for j=1:length-i
        if openlist(j,4)>openlist(j+1,4)
            temp=openlist(j,4);
            openlist(j,4)=openlist(j+1,4);
            openlist(j+1,4)=temp;
        end
    end
end

   
end

