function [Path,distanceX,OPEN_num]= Astar_G_du1(Obs_Closed,St,Ta,MAX_X,MAX_Y)
% Astar_G_du1: 改进A* + B样条平滑全局规划 (完整修复版)

xStart = St(1,1);      
yStart = St(1,2); 
xTarget = Ta(1,1);    
yTarget = Ta(1,2);
CLOSED = Obs_Closed;
Num_obs = size(Obs_Closed,1); 

% 算法初始化
MAX_NODES = MAX_X * MAX_Y;  
OPEN=zeros(MAX_NODES, 8);   
CLOSED_BUFFER = zeros(MAX_NODES, 2); 
CLOSED_BUFFER(1:Num_obs, :) = CLOSED; 
CLOSED = CLOSED_BUFFER; 
CLOSED_COUNT=Num_obs;   
Nobs=Num_obs;
xNode=xStart;      
yNode=yStart;      
OPEN_COUNT=1;    
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);                                                                                    
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);  
OPEN(OPEN_COUNT,1)=1;      
CLOSED_COUNT=CLOSED_COUNT+1;  
CLOSED(CLOSED_COUNT,1)=xNode; 
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

% 数值稳定性修正
P_obsNT=Obs_array(xStart,yStart,xTarget,yTarget,CLOSED(1:Num_obs,:),Nobs); 
P=log(P_obsNT + 1); 

% --- A* 主循环 ---
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)       
    exp_array=expand_array_Obs8(xNode,yNode,path_cost,xTarget,yTarget,CLOSED(1:CLOSED_COUNT,:),MAX_X,MAX_Y,Nobs,P);  
    exp_count=size(exp_array,1);   
    
    for i=1:exp_count         
        flag=0;                
        for j=1:OPEN_COUNT     
            if(OPEN(j,1)==1 && exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )    
                OPEN(j,8)=min(OPEN(j,8),exp_array(i,5));                       
                if OPEN(j,8)== exp_array(i,5)                                  
                    OPEN(j,4)=xNode;
                    OPEN(j,5)=yNode;
                    OPEN(j,6)=exp_array(i,3);
                    OPEN(j,7)=exp_array(i,4);
                end;
                flag=1;                    
            end;
        end;
        
        if flag == 0
            OPEN_COUNT = OPEN_COUNT+1; 
            OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
            OPEN(OPEN_COUNT,1)=1; 
        end;
    end;
    
    index_min_node = min_fn(OPEN(1:OPEN_COUNT,:),OPEN_COUNT,xTarget,yTarget);   
    if (index_min_node ~= -1)    
        xNode=OPEN(index_min_node,2);
        yNode=OPEN(index_min_node,3);
        path_cost=OPEN(index_min_node,6);
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        OPEN(index_min_node,1)=0; 
    else
        NoPath=0;
    end;
end;

% --- 回溯路径与优化 ---
if NoPath == 1
    % 1. 基础路径回溯
    i=CLOSED_COUNT;    
    Optimal_path=[];     
    xval=CLOSED(i,1);    
    yval=CLOSED(i,2);
    i=1;
    Optimal_path(i,1)=xval; 
    Optimal_path(i,2)=yval;
                
    if ( (xval == xTarget) && (yval == yTarget))  
       Target_ind=node_index(OPEN(1:OPEN_COUNT,:),xval,yval);
       parent_x=OPEN(Target_ind,4); 
       parent_y=OPEN(Target_ind,5);
       Optimal_path(i,3)=parent_x; 
       Optimal_path(i,4)=parent_y;
       
       while( parent_x ~= xStart || parent_y ~= yStart)   
           i=i+1; 
           Optimal_path(i,1) = parent_x;             
           Optimal_path(i,2) = parent_y;
           inode=node_index(OPEN(1:OPEN_COUNT,:),parent_x,parent_y); 
           Optimal_path(i,3) = OPEN(inode,4);
           Optimal_path(i,4) = OPEN(inode,5);
           parent_x=Optimal_path(i,3);
           parent_y=Optimal_path(i,4);
       end;
    end
    Num_Opt=size(Optimal_path,1);
 
    % 2. 第一次折线优化
    Valid_CLOSED = CLOSED(1:CLOSED_COUNT, :); 
    Optimal_path_one=Line_OPEN_ST(Optimal_path,Valid_CLOSED,Num_obs,Num_Opt);
 
    Optimal_path_try=[1 1 1 1];
    Optimal_path_new=[1 1 ];
    i=1;q=1;
    x_g=Optimal_path_one(Num_Opt,3);y_g=Optimal_path_one(Num_Opt,4);
 
    Optimal_path_try(i,1)= Optimal_path_one(q,1);
    Optimal_path_try(i,2)= Optimal_path_one(q,2);
    Optimal_path_try(i,3)= Optimal_path_one(q,3);
    Optimal_path_try(i,4)= Optimal_path_one(q,4);
 
    % 死循环保护
    safety_break = 0;
    while (Optimal_path_try(i,3)~=x_g || Optimal_path_try(i,4)~=y_g)
       safety_break = safety_break + 1;
       if safety_break > 5000
           warning('A* 路径优化回溯陷入死循环，强制跳出');
           break;
       end
       i=i+1;
       q=Optimal_index(Optimal_path_one,Optimal_path_one(q,3),Optimal_path_one(q,4));
       Optimal_path_try(i,1)= Optimal_path_one(q,1);
       Optimal_path_try(i,2)= Optimal_path_one(q,2);
       Optimal_path_try(i,3)= Optimal_path_one(q,3);
       Optimal_path_try(i,4)= Optimal_path_one(q,4);
    end
    n=size(Optimal_path_try,1);
    for i=1:1:n
        Optimal_path_new(i,1)=Optimal_path_try(n,3);
        Optimal_path_new(i,2)=Optimal_path_try(n,4);
        n=n-1;
    end
    num_op=size(Optimal_path_new,1)+1;
    Optimal_path_new(num_op,1)=Optimal_path_try(1,1);
    Optimal_path_new(num_op,2)=Optimal_path_try(1,2);
 
    % 3. 第二次折线优化
    Optimal_path_two=Line_OPEN_STtwo(Optimal_path_new,Valid_CLOSED,Num_obs,num_op);
    num_optwo=size(Optimal_path_two,1)+1;
    Optimal_path_two(num_optwo,1)=xStart;
    Optimal_path_two(num_optwo,2)=yStart;
 
    j=num_optwo;
    Optimal_path_two2=[xStart yStart];
    for i=1:1:num_optwo
        Optimal_path_two2(i,1)=Optimal_path_two(j,1);
        Optimal_path_two2(i,2)=Optimal_path_two(j,2);
        j=j-1;
    end
    
    % 4. 第三次折线优化 (生成变量 Optimal_path_three)
    Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,Valid_CLOSED,Num_obs,num_optwo);
  
    % 补全终点
    if ~isempty(Optimal_path_three) && (Optimal_path_three(end,1)~=xTarget || Optimal_path_three(end,2)~=yTarget)
        Optimal_path_three = [Optimal_path_three; xTarget, yTarget];
    end
    
    % --- 【以下是B样条和平滑的修复代码】 ---

    % 1. 确保路径是从 起点 -> 终点 的顺序
    % 防止 DWA 倒着走导致绕圈
    dist_start = norm(Optimal_path_three(1,:) - [xStart, yStart]);
    dist_end = norm(Optimal_path_three(end,:) - [xStart, yStart]);
    if dist_start > dist_end
        % 如果起点离Start很远，说明数组反了，翻转一下
        Optimal_path_three = flipud(Optimal_path_three);
    end

    % 2. 路径加密 (步长 1.0)
    Polyline_Path = Optimal_path_three; 
    Dense_Path = Subdivide_Path(Polyline_Path, 1.0);
    
    % 3. 生成三次B样条
    % 注意：插值点数量取决于你的路径长度，太少会不圆滑，太多计算量大
    BSpline_Path = Generate_B_Spline_Path(Dense_Path, 15);
    
    % 4. 【关键步骤】清理路径：删除重复点和过近点
    % 解决 B样条导致的重叠点问题
    NewOptimal_path = Clean_Path(BSpline_Path, 0.05); 
    
    % 5. 再次确保终点精确
    if norm(NewOptimal_path(end,:) - [xTarget, yTarget]) > 0.1
        NewOptimal_path = [NewOptimal_path; xTarget, yTarget];
    end
    
    % --- 输出结果 ---
    Path = NewOptimal_path;
    
    S=0;
    for i=1:1:(size(NewOptimal_path,1)-1)
        Dist=sqrt( ( NewOptimal_path(i,1) - NewOptimal_path(i+1,1) )^2 + ( NewOptimal_path(i,2) - NewOptimal_path(i+1,2))^2);
        S=S+Dist;
    end
    distanceX = S;
    OPEN_num = size(OPEN,1); 
else
    Path = [];
    distanceX = 0;
    OPEN_num = 0;
end
end

% =========================================================================
% 子函数1：路径加密 (已优化)
% =========================================================================
function dense_path = Subdivide_Path(original_path, step_size)
    if isempty(original_path)
        dense_path = [];
        return;
    end
    dense_path = original_path(1, :); 
    
    for i = 1 : size(original_path, 1) - 1
        p1 = original_path(i, :);
        p2 = original_path(i+1, :);
        dist = norm(p2 - p1);
        
        if dist > 1e-3 % 忽略极小距离
            num_segments = ceil(dist / step_size);
            if num_segments > 1
                x_vals = linspace(p1(1), p2(1), num_segments + 1)';
                y_vals = linspace(p1(2), p2(2), num_segments + 1)';
                new_points = [x_vals(2:end), y_vals(2:end)];
                dense_path = [dense_path; new_points];
            else
                dense_path = [dense_path; p2];
            end
        end
    end
end

% =========================================================================
% 子函数2：三次B样条生成 (保持不变)
% =========================================================================
function [smooth_path] = Generate_B_Spline_Path(path_points, num_interp)
    n = size(path_points, 1);
    if n < 3
        smooth_path = path_points; 
        return;
    end
    % 这里的 repmat 会产生重复点，后续必须用 Clean_Path 清理
    P_aug = [repmat(path_points(1,:), 2, 1); path_points; repmat(path_points(end,:), 2, 1)];
    M = [ -1  3 -3  1; 3 -6  3  0; -3  0  3  0; 1  4  1  0 ] / 6;
    smooth_path = [];
    n_aug = size(P_aug, 1);
    for i = 1 : (n_aug - 3)
        P_segment = P_aug(i:i+3, :);
        t_vals = linspace(0, 1, num_interp)';
        T_matrix = [t_vals.^3, t_vals.^2, t_vals, ones(size(t_vals))];
        pts = T_matrix * M * P_segment;
        smooth_path = [smooth_path; pts];
    end
end

% =========================================================================
% 子函数3：路径清理函数 (删除重复点)
% =========================================================================
function clean_path = Clean_Path(path, min_dist)
    if isempty(path)
        clean_path = [];
        return;
    end
    clean_path = path(1, :);
    last_idx = 1;
    for i = 2:size(path, 1)
        dist = norm(path(i, :) - path(last_idx, :));
        % 只有距离大于阈值才加入
        if dist > min_dist
            clean_path = [clean_path; path(i, :)];
            last_idx = i;
        end
    end
end