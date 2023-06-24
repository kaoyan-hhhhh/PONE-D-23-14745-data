%传入节点的F值，将节点的F值插入到数组末尾
%openlist:open列表存放节点和节点的F值
%node_coordinate:要插入的节点坐标
%node_f_data:要插入的节点的f值
function openlist=queue_remove(openlist)

%删除堆顶元素
open_size=size(openlist);
last=open_size(1);
head=1;
openlist(head,:)=openlist(last,:);
%去掉最后一行元素
openlist(end,:)=[];
length=last-1;
%冒泡排序，从小到大
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