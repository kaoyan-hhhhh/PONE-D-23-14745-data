%传入节点的F值，将节点的F值插入到数组末尾
%openlist:open列表存放节点和节点的F值
%node_f_data:要插入的节点的f值
function openlist=Binary_heap_insert(openlist,x,y,z,node_f_data)

%新增节点添加到尾部
open_size=size(openlist);
row=open_size(1);
last=row+1;
openlist(last,:)=[x,y,z,node_f_data];

%调整堆结构
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

