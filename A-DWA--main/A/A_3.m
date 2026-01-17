function NewOptimal_path=A_2(MAX,xStart,yStart,xTarget,yTarget,CLOSED,hui)
% A*算法 + 适度路径加密 + 三次B样条平滑
% 修改记录：
% 1. 增大了 Subdivide_Path 的步长，从 0.5 改为 1.5。
% 效果：控制点变稀疏，曲线拐弯更圆滑，不再僵硬。
MAX_X=size(MAX,2);
MAX_Y=size(MAX,1);
axis([1 MAX_X+1, 1 MAX_Y+1])
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
'xGrid','on','yGrid','on')
grid on;
hold on;
for j=1:MAX_X
for i=1:MAX_Y
if (MAX(i,j)==1)
fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');
end
end
end
Num_obs=size(CLOSED,1);
tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
P_obsNT=Obs_array(xStart,yStart,xTarget,yTarget,CLOSED(1:Num_obs,:),Nobs);
P=log(P_obsNT);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START A* ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% 回溯路径
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
while (Optimal_path_try(i,3)~=x_g || Optimal_path_try(i,4)~=y_g)
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
Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,Valid_CLOSED,Num_obs,num_optwo);
num_opthree=size(Optimal_path_three,1)+1;
Optimal_path_three(num_opthree,1)=xTarget;
Optimal_path_three(num_opthree,2)=yTarget;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 【核心升级】：三次B样条平滑 (更圆滑版)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. 获取原始A*折线路径
Polyline_Path = Optimal_path_three;
% 2. 【关键调整】适度加密路径
% 之前是 0.5 (太密，导致僵硬)，现在改为 1.5
% 含义：每隔 1.5 个单位距离加一个点。
% 效果：点变稀疏了，B样条的平滑特性就能发挥出来，拐弯会更圆。
% 如果你觉得还不够圆，可以改成 2.0；如果觉得切角太大怕撞墙，改回 1.0。
Dense_Path = Subdivide_Path(Polyline_Path, 3);
% 3. 生成三次B样条平滑路径
% 插值点数设为 10~20 足够画出光滑曲线
BSpline_Path = Generate_B_Spline_Path(Dense_Path, 15);
% 4. 更新最终路径
NewOptimal_path = BSpline_Path;
disp('A_3 (B样条版) 规划时间')
toc
% 计算转折度数 (使用原始折线计算)
j = size(Polyline_Path,1);
angle_du=0;
for i=1:1:(j-2)
du=angle6(Polyline_Path(i,1),Polyline_Path(i,2),Polyline_Path(i+1,1) ,Polyline_Path(i+1,2),Polyline_Path(i+2,1) ,Polyline_Path(i+2,2));
angle_du=angle_du+du;
end
disp('A_3 转折度数')
angle_du
disp('A_3 转折次数')
ci = j - 2
S=0;
% 计算B样条平滑路径的长度
for i=1:1:(size(BSpline_Path,1)-1)
Dist=sqrt( ( BSpline_Path(i,1) - BSpline_Path(i+1,1) )^2 + ( BSpline_Path(i,2) - BSpline_Path(i+1,2))^2);
S=S+Dist;
end
disp('A_3 平滑路径长度')
S
disp('A_3 遍历节点数')
op_size=OPEN_COUNT
if hui==1
for n=1:op_size
if OPEN(n,1) == 0 continue; end
i=OPEN(n,2);
j=OPEN(n,3);
fill([i,i+1,i+1,i],[j,j,j+1,j+1],[0.85 0.85 0.85]);
end
end
plot(xTarget+0.5,yTarget+0.5,'go');
plot(xStart+0.5,yStart+0.5,'b^');
% 绘制A*折线 (青色虚线)
plot(Polyline_Path(:,1)+.5,Polyline_Path(:,2)+.5,'c--','linewidth',1);
% 绘制B样条平滑曲线 (蓝色实线)
plot(BSpline_Path(:,1)+.5,BSpline_Path(:,2)+.5,'b','linewidth',2);
xlabel('基于A_3算法(Cubic B-Spline)的路径规划 ','Color','black');
else
pause(1);
h=msgbox('Sorry, No path exists to the Target!','warn');
uiwait(h,5);
end
end % 主函数结束
% =========================================================================
% 子函数1：路径线性插值加密 (调整版)
% =========================================================================
function dense_path = Subdivide_Path(original_path, step_size)
% original_path: 原始路径点 [x, y]
% step_size: 插值步长，越大则点越稀疏，曲线越平滑，但越容易切角
if isempty(original_path)
dense_path = [];
return;
end
dense_path = original_path(1, :); % 放入起点
for i = 1 : size(original_path, 1) - 1
p1 = original_path(i, :);
p2 = original_path(i+1, :);
dist = norm(p2 - p1);
% 如果两点距离大于步长，则进行插值
if dist > step_size
num_segments = ceil(dist / step_size);
x_vals = linspace(p1(1), p2(1), num_segments + 1)';
y_vals = linspace(p1(2), p2(2), num_segments + 1)';
% 去掉第一个点，添加剩下的点
new_points = [x_vals(2:end), y_vals(2:end)];
dense_path = [dense_path; new_points];
else
% 距离小于步长，不加密，直接把转折点加进去
% 这样可以保证转折处的控制点不会堆积在一起，利于平滑
dense_path = [dense_path; p2];
end
end
end
% =========================================================================
% 子函数2：三次B样条曲线生成 (Cubic B-Spline)
% =========================================================================
function [smooth_path] = Generate_B_Spline_Path(path_points, num_interp)
% path_points: 控制点 [x, y]
% num_interp: 每个片段生成的插值点数量
n = size(path_points, 1);
if n < 3
smooth_path = path_points;
return;
end
% 重复首尾控制点，保证曲线经过起点和终点
P_aug = [repmat(path_points(1,:), 2, 1); path_points; repmat(path_points(end,:), 2, 1)];
% 三次B样条基矩阵
M = [ -1 3 -3 1;
3 -6 3 0;
-3 0 3 0;
1 4 1 0 ] / 6;
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