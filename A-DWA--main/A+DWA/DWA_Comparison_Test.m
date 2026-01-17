clc
clear
close all; 
figure 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                地图建模
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MAX0= [      0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 1 1 0 0 1 0 0 0 0 1 0 0 0 0 0 0
             0 0 0 0 0 1 0 0 1 0 0 0 1 1 0 0 0 0 0 0
             0 0 0 0 0 1 0 1 0 1 0 0 0 1 0 0 0 0 0 0
             0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 1 1 1 0 0 0 1 1 1 1 0 1 0 0 0 1 1 0 0
             0 0 0 0 0 0 0 1 0 0 1 0 1 0 0 0 1 1 0 0
             0 0 1 0 0 0 0 0 0 1 0 1 0 0 0 0 1 1 0 0
             0 0 1 1 0 0 0 1 1 1 1 0 1 0 0 0 1 1 0 0
             0 0 1 0 0 0 0 1 1 0 0 0 1 0 0 0 1 1 0 0 
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 1 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 0
             0 1 0 0 1 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 
             0 1 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 
             0 0 0 0 0 0 0 1 0 0 0 1 1 0 0 0 0 0 0 0
             0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 1 0 0 0 0 
             1 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 
             1 0 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 
             1 1 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 ] ;

MAX=rot90(MAX0,3);      
MAX_X=size(MAX,2);      
MAX_Y=size(MAX,1);      
MAX_VAL=10;             
x_val = 1;
y_val = 1;
axis([1 MAX_X+1, 1 MAX_Y+1])                
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')
grid on;                                   
hold on;                                   
n=0;
k=1;          
CLOSED=[];

%% 读取地图障碍物
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          % 绘制黑色方块
          fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  
          CLOSED(k,1)=i;  
          CLOSED(k,2)=j; 
          k=k+1;
        end
    end
end
% 防止没有障碍物时报错
if isempty(CLOSED)
    CLOSED = [-100, -100];
end
Area_MAX(1,1)=MAX_X;
Area_MAX(1,2)=MAX_Y;
Obs_Closed=CLOSED; 
num_Closed=size(CLOSED,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                       设置起始点和多个目标点
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 设置起始点、目标点
%%%   选择目标位置
pause(0.5);                                  
h=msgbox('请使用鼠标左键选择巡检USV位置');          
uiwait(h,5);                               
if ishandle(h) == 1                        
    delete(h);
end
xlabel('请使用鼠标左键选择巡检USV位置','Color','black');  
but=0;
while (but ~= 1) 
    [xval,yval,but]=ginput(1);                                 
end
xval=floor(xval);                                             
yval=floor(yval);
xTarget=xval;
yTarget=yval;
plot(xval+.5,yval+.5,'go');                                    
text(xval+1,yval+1,'Target')                                  

%%%   选择起始位置
h=msgbox('请选择USV机器人初始起点');                   
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('请使用鼠标左键选择USV终点 ','Color','black');                
but=0;
while (but ~= 1) 
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;
yStart=yval;
plot(xval+.5,yval+.5,'b^');
text(xval+1,yval+1,'Start')  
xlabel('起始点位置标记为 △ ，目标点位置标记为 o ','Color','black'); 

Start=[xStart yStart];
Goal=[xTarget yTarget];

%% A* 全局规划
[Line_path, distance_x, OPEN_num] = Astar_G_du(Obs_Closed, Start, Goal, MAX_X, MAX_Y);
% 传统对比时，通常需要看到A*的参考线，这里保留虚线代码但如果想去掉可注释
% plot(Line_path(:,1)+.5, Line_path(:,2)+.5, 'b:', 'linewidth', 2);

plot(xStart+.5,yStart+.5,'b^');
plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo');   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                       虚拟障碍物设置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Obst_d_d_line = repmat([-100, -100], 5000, 1); 
Obs_d_j = [-100, -100];       

%% 【关键修改】机器人运动学模型 (传统/激进模式)
angle_node=pi/4;

% Kinematic = [最高速度, 最高旋转速度, 加速度, 旋转加速度, ...]
% 修改：增大了旋转速度(40度)和旋转加速度(100度)，让它能做急转弯
Kinematic=[1.0, toRadian(40.0), 0.5, toRadian(100.0), 0.02, toRadian(1)];

% 【关键修改】评价函数系数 [heading, dist, velocity, predictDT]
% 1. Heading (0.1 -> 0.2): 增大，更执着于走直线去目标。
% 2. Dist (0.2 -> 0.05): 大幅减小，让机器人不怕死，敢贴着墙走。
% 3. PredictDT (3.0 -> 1.0): 减小预测时间，让机器人变得“目光短浅”，直到快撞上才避障。
evalParam=[0.2, 0.05, 0.2, 1.0]; 

path_node=Line_path;

% 运行DWA
Result_x=DWA_ct_dong(Obs_Closed,Obst_d_d_line,Obs_d_j,Area_MAX,Goal,Line_path,path_node,Start,angle_node,Kinematic,evalParam);

%%%%%%%%%%% 画图
figure 
axis([1 MAX_X+1, 1 MAX_Y+1])                
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
        'xGrid','on','yGrid','on');   
grid on;       
hold on;

% 绘制障碍物
num_obc=size(Obs_Closed,1);
for i_obs=1:1:num_obc
    x_obs=Obs_Closed(i_obs,1);
    y_obs=Obs_Closed(i_obs,2);
    if x_obs > 0 && y_obs > 0
        fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
    end
end

% 绘制起点终点
plot(xStart+.5,yStart+.5,'b^');
plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo');
  
% 绘制实际路径 (蓝色实线)
num_x=size(Result_x,1);
Result_plot=[Result_x;Goal(1,1) Goal(1,2) Result_x(num_x,3) 0 0];
plot(Result_x(:,1)+0.5, Result_x(:,2)+0.5,'b','linewidth',2);hold on;

num_p=num_x+1;
ti=1:1:num_p;

figure
plot(ti,Result_plot(:,3),'-b');hold on;
legend('姿态角度')

figure
plot(ti,Result_plot(:,4),'-b');hold on;
plot(ti,Result_plot(:,5),'-r');hold on;
legend('线速度','角速度')

% 计算路径长度
S=0;
for i=1:1:num_x  
    Dist=sqrt( ( Result_plot(i,1) - Result_plot(i+1,1) )^2 + ( Result_plot(i,2) - Result_plot(i+1,2))^2);
    S=S+Dist;
end
disp('路径长度')
S

% 计算并输出运行时间
dt = 0.1; 
Travel_Time = num_x * dt; 
disp('运行时间(s)')
Travel_Time