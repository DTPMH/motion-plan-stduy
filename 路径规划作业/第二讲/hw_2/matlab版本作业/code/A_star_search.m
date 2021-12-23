function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);%返回的是map矩阵的行数
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));%创建10*10的全1矩阵
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;%目标的MAP值为0
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;%障碍物的MAP值为-1
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;%起始点的MAP值为1

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];
    path=[];
    path_finally=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end%将地图中的障碍点放入CLOSED矩阵中
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
goal=[xTarget,yTarget];
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       cout=0;
    while(cout<200) %you have to dicide the Conditions for while loop exit 
        cout=cout+1;
                [idex,goalinopen]=inopen(goal,OPEN);
                if(goalinopen)
            path=[path;OPEN(idex,:)];
            break;  
                end
        [Y,I]=sort(OPEN(:,8));
        OPEN=OPEN(I,:);
        current_node=OPEN(1,:);%将f值最小的设置为当前节点
        OPEN(1,:)=[];
        path=[path;current_node];
        neighbor_node=search_neighbor(current_node);
        for i=1:size(neighbor_node,1)
            [idex,isinopen]=inopen(neighbor_node(i,:),OPEN);
            isinpath=inpath(neighbor_node(i,:),path);
            h1=h(neighbor_node(i,:),xTarget,yTarget);
            g=neighbor_node(i,3);
            f=g+h1;
            open_node=[1,neighbor_node(i,1),neighbor_node(i,2),current_node(2),current_node(3),h1,g,f];
            if(neighbor_node(i,1)>=MAX_X||neighbor_node(i,2)>=MAX_Y||neighbor_node(i,1)<=0||neighbor_node(i,2)<=0)
                continue;
            elseif(MAP(neighbor_node(i,1),neighbor_node(i,2))==-1)
                continue;
            elseif(isinpath)
                continue;
            elseif(isinopen)
               if((neighbor_node(i,3)+current_node(7))<OPEN(idex,7))
                   open_node(4:5)=current_node(1,2:3);
                   OPEN(idex,:)=open_node;
               end
            else
               OPEN=[OPEN;open_node];
            end
        end
    end %End of While Loop  
    path_node=[1,1];
    path_finally=[path_finally;path_node];
 for j=1:size(path,1)
     if(path(j,4)==path_node(1)&&path(j,5)==path_node(2))
         path_node=path(j,2:3);
         path_finally=[path_finally;path_node];
     end
 end
% path=path_finally;
end
