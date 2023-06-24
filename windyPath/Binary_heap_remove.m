%删除F值最小的节点，调整堆结构
%openlist:open列表存放节点和节点的F值
function openlist=Binary_heap_remove(openlist)

%删除堆顶元素
open_size=size(openlist);
last=open_size(1);
head=1;
openlist(head,:)=openlist(last,:);
%去掉最后一行元素
openlist(end,:)=[];
%last=last-1;

while head*2+1<=last-1
    child1=head*2;
    child2=head*2+1;
    if openlist(child1,4)<openlist(child2,4)
        child_min=child1;
    else
        child_min=child2;
    end
    if openlist(head,4)<=openlist(child_min,4)
        break;
    end
    temp=openlist(head,:);
    openlist(head,:)=openlist(child_min,:);
    openlist(child_min,:)=temp;
    head=child_min;
        
    
end


end