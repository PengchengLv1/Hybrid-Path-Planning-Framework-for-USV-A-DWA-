clc; clear;

% 1. 读取图片 (请确保图片文件名正确，并且在当前目录下)
% 如果你的图片叫 '二值图.png'，请修改下面这行
try
    img = imread('3.png'); 
catch
    error('找不到图片文件！请确认图片在当前文件夹，且文件名正确。');
end

% 2. 如果是彩色图，转为灰度图
if size(img, 3) == 3
    img = rgb2gray(img);
end

% 3. 强制缩放图片到 50x50 像素
% 使用 'nearest' (最近邻插值) 可以保持边缘锐利，避免出现小数
img_resized = imresize(img, [50, 50], 'nearest');

% 4. 转换为 0/1 矩阵
% 图片中：黑色(数值接近0)是障碍物，白色(数值接近255)是空地
% 我们需要：黑色变1，白色变0
% 所以逻辑是：像素值小于 128 的变成 1 (障碍)
MAX0 = double(img_resized < 128);

% 5. 强制清空起点和终点 (防止生成的地图把路堵死)
% 清空左上角 (1,1) 附近
MAX0(1:3, 1:3) = 0;
% 清空右下角 (50,50) 附近
MAX0(end-2:end, end-2:end) = 0;

% 6. 显示转换后的效果 (预览图)
figure(1);
subplot(1,2,1); imshow(img); title('原图');
subplot(1,2,2); imshow(1-MAX0); title('50x50 栅格化后 (黑=障碍)');

% 7. 【关键】将矩阵打印到命令行，方便你复制
fprintf('请复制下方矩阵替换你的代码：\n\n');
fprintf('MAX0 = [\n');
for i = 1:50
    for j = 1:50
        fprintf('%d ', MAX0(i,j));
    end
    if i < 50
        fprintf(';\n'); % 换行
    else
        fprintf('];\n'); % 最后一行结束
    end
end