function [Path,distanceX,OPEN_num]= Astar_G_du1(Obs_Closed,St,Ta,MAX_X,MAX_Y)
% Astar_G_du1: 改进A* + B样条平滑全局规划
% 请确保当前目录下有: Obs_array.m, expand_array_Obs8.m, insert_open.m, min_fn.m 等辅助文件

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

% 【修复1】数值稳定性修正：加1防止 log(0)
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

% --- 回溯路径 ---
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
  Num_Opt=size(Optimal_path,1);
 
  % 1. 第一次折线优化
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
 
   % 【修复2】死循环保护
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
 
 % 2. 第二次折线优化
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
 % 3. 第三次折线优化
  Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,Valid_CLOSED,Num_obs,num_optwo);
  
  % 【修复3】删除强制添加 Target 的代码，防止重复点
  % 如果末端没有到达 Target，可考虑自动补全，但 Line_OPEN_STtwo 通常已包含
  if ~isempty(Optimal_path_three) && (Optimal_path_three(end,1)~=xTarget || Optimal_path_three(end,2)~=yTarget)
      Optimal_path_three = [Optimal_path_three; xTarget, yTarget];
  end
 
 % 4. 【核心】：三次B样条平滑 (圆滑且防撞)
 Polyline_Path = Optimal_path_three; 
 
 % 适度加密路径 (步长 1.5 - 保证圆滑)
 Dense_Path = Subdivide_Path(Polyline_Path, 1.5);
 
 % 生成三次B样条平滑路径 (插值点 15)
 BSpline_Path = Generate_B_Spline_Path(Dense_Path, 15);
 
 NewOptimal_path = BSpline_Path; 
 
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
% 子函数1：路径加密 (修复重复点问题)
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
        
        % 【修复】只有当两点距离足够大时才处理，防止 0 距离重复点
        if dist > 1e-4
            if dist > step_size
                num_segments = ceil(dist / step_size);
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
% 子函数2：三次B样条生成
% =========================================================================
function [smooth_path] = Generate_B_Spline_Path(path_points, num_interp)
    n = size(path_points, 1);
    if n < 3
        smooth_path = path_points; 
        return;
    end
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