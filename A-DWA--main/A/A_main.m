%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 改进A* (A_2) 与 传统A* (A_1) 与 平滑A* (A_3) 算法比较
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear;
figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 算法开关设置
% 1 = 开启, 0 = 关闭
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A=1;      % 传统 A* (A_1)
Dij_A=1;  % 是否显示搜索过程栅格

A_g=1;    % 改进 A* (A_2, 折线优化)
Dij_Ag=0; % 建议关闭显示，以免覆盖

A_s=1;    % [新增] 平滑 A* (A_3, B样条优化)
Dij_As=0; 

%% 设置地图 (保持你原来的地图不变)
MAX0= [      0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 0 0 1
             0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 1 0 0 1
             0 0 0 0 1 0 0 0 0 0 1 0 1 0 1 1 0 1 0 0
             0 0 0 1 1 0 0 0 1 1 1 0 1 0 1 0 1 0 0 0
             0 0 0 0 0 0 0 0 1 1 1 0 1 0 1 0 1 1 0 0 
             0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0
             0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 
             1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] ;

MAX=MAX0;      
MAX_X=size(MAX,2);                                
MAX_Y=size(MAX,1);                                
axis([1 MAX_X+1, 1 MAX_Y+1])                
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')
grid on;                                   
hold on;                                   
n=0;
k=1;          
CLOSED=[];
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  
          CLOSED(k,1)=i;  
          CLOSED(k,2)=j; 
          k=k+1;
        end
    end
end

%% 设置起始点、目标点
pause(0.5);                                                    
h=msgbox('请使用鼠标左键选择USV位置');                       
uiwait(h,5);                                                   
if ishandle(h) == 1                                            
    delete(h);
end
xlabel('请使用鼠标左键选择USV位置','Color','black');          
but=0;
while (but ~= 1)                                               
    [xval,yval,but]=ginput(1);                                 
end
xval=floor(xval);                                              
yval=floor(yval);
xTarget=xval;                                                  
yTarget=yval;                                                  
plot(xval+.5,yval+.5,'ro');                                    
text(xval+1,yval+1,'Target')                                   

h=msgbox('请使用鼠标左键选择USV初始位置');               
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('请选择USV初始位置 ','Color','black');           
but=0;
while (but ~= 1)                                              
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;                                                  
yStart=yval;                                                  
plot(xval+.5,yval+.5,'y*');
text(xval+1,yval+1,'Start')  
xlabel('正在进行路径规划...','Color','black'); 
drawnow;

%% 1. 传统 A* 算法 (A_1)
if  A==1
   disp('正在运行 A_1 (传统A*)...');
   A1_path=A_1(MAX,xStart,yStart,xTarget,yTarget,CLOSED,Dij_A) ; 
end

%% 2. 改进 A* 算法 (A_2 - 折线优化)
if  A_g==1
   disp('正在运行 A_2 (折线优化)...');
   % 注意：这里 Dij_Ag 通常设为0，避免重复画栅格
   A2_path=A_2(MAX,xStart,yStart,xTarget,yTarget,CLOSED,Dij_Ag) ;  
end

%% 3. [新增] B样条平滑算法 (A_3 - 曲线平滑)
if A_s==1
    disp('正在运行 A_3 (B样条平滑)...');
    % 调用新写的 A_3 函数
    A3_path = A_3(MAX,xStart,yStart,xTarget,yTarget,CLOSED,Dij_As);
end

%% 最终综合画图比较
% f=1 将所有路径放在一个地图里面
f=1;
if f==1
    figure('Name','算法对比结果'); % 新开一个窗口
    
    % 重画地图背景
    axis([1 MAX_X+1, 1 MAX_Y+1])
    set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
        'xGrid','on','yGrid','on')
    grid on; hold on;
    
    % 画障碍物
    for j=1:MAX_X
        for i=1:MAX_Y
            if (MAX(i,j)==1)
              fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k'); 
            end
        end
    end
    
    % 画起点终点
    plot(xTarget+.5,yTarget+.5,'ro','MarkerSize',10,'LineWidth',2); 
    text(xTarget+1,yTarget+1,'Target');
    plot(xStart+.5,yStart+.5,'y*','MarkerSize',10,'LineWidth',2);
    text(xStart+1,yStart+1,'Start');
    
    legend_str = {}; %用于存储图例文字
    
    % --- 绘制 A_1 (传统) ---
   % --- 绘制 A_1 (传统) ---
    if  A==1 && ~isempty(A1_path)
       % 颜色：浅绿色 (RGB: [0.5, 0.9, 0.5])
       plot(A1_path(:,1)+.5,A1_path(:,2)+.5, 'LineStyle', '-', ...
            'Color', [0.5, 0.9, 0.5], 'linewidth', 2); 
       legend_str{end+1} = 'A_1: 传统A*';
    end
    
    % --- 绘制 A_2 (折线优化) ---
    if  A_g==1 && ~isempty(A2_path)
       % 颜色：天蓝色/浅蓝 (RGB: [0.4, 0.7, 1.0])
       plot(A2_path(:,1)+.5,A2_path(:,2)+.5, 'LineStyle', '-', ...
            'Color', [0.4, 0.7, 1.0], 'linewidth', 2.5); 
       legend_str{end+1} = 'A_2: 改进A*(折线)';
    end
    
    % --- 绘制 A_3 (B样条平滑) ---
    if A_s==1 && ~isempty(A3_path)
        % 颜色：淡红色/橘粉色 (RGB: [1.0, 0.5, 0.5])
        plot(A3_path(:,1)+.5,A3_path(:,2)+.5, 'LineStyle', '-', ...
             'Color', [1.0, 0.5, 0.5], 'linewidth', 1.5); 
        legend_str{end+1} = 'A_3: 改进A*(B样条平滑)';
    end
    
    legend(legend_str, 'Location', 'best');
    title('三种路径规划算法对比');
    xlabel('浅绿:传统A* | 浅蓝:改进折线A* | 浅红:B样条平滑A*');
end