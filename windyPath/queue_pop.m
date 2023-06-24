%传入节点的F值，将节点的F值插入到数组末尾
%openlist:open列表存放节点和节点的F值
%node_coordinate:要插入的节点坐标
%node_f_data:要插入的节点的f值
function [xcurrent, ycurrent, zcurrent]=queue_pop(openlist)
xcurrent=openlist(1,1);
ycurrent=openlist(1,2);
zcurrent=openlist(1,3);

end