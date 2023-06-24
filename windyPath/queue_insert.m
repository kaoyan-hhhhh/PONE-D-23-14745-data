%传入节点的F值，将节点的F值插入到数组末尾
%openlist:open列表存放节点和节点的F值
%node_coordinate:要插入的节点坐标
%node_f_data:要插入的节点的f值
function openlist=queue_insert(openlist,x,y,z,node_f_data)

%新增节点添加到尾部
open_size=size(openlist);
row=open_size(1);
last=row+1;
openlist(last,:)=[x,y,z,node_f_data];
%获取open长度
length=last;
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

