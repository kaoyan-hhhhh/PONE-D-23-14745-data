%����ڵ��Fֵ�����ڵ��Fֵ���뵽����ĩβ
%openlist:open�б��Žڵ�ͽڵ��Fֵ
%node_coordinate:Ҫ����Ľڵ�����
%node_f_data:Ҫ����Ľڵ��fֵ
function openlist=queue_remove(openlist)

%ɾ���Ѷ�Ԫ��
open_size=size(openlist);
last=open_size(1);
head=1;
openlist(head,:)=openlist(last,:);
%ȥ�����һ��Ԫ��
openlist(end,:)=[];
length=last-1;
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