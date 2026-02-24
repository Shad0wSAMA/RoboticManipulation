clc, clear;
figure; hold on; axis equal; grid on;
xlim([0 9]); ylim([-9 9]);
lw = 6; 

% 定义缩放因子
scale = 0.7;

% 复合变换函数：(旋转 + 映射 + 整体缩小 0.7)
% 这里的 +2 是你代码中带有的偏移量，确保字在 X 轴的显示范围
finalX = @(x, y) ((-y + 10) * 0.9 + 2) * scale + 2.5;
finalY = @(x, y) ((x * 0.9) - 4.5) * scale;

% 定义原始笔画数据 (x_start, x_end; y_start, y_end)
strokes = {
    [2 3], [9 9];       % 左-上横
    [1.5 3.5], [8 8];   % 左-中横
    [2.5 2.5], [4 8];   % 左-竖
    [1.5 2], [5 7];     % 左-撇
    [3.5 3], [6 7];     % 左-点
    [4.5 6.5], [9 9];   % 右-顶横
    [4.5 6.5], [8 8];   % 右-口上横
    [4.5 4.5], [7 8];   % 右-口左竖
    [6.5 6.5], [7 8];   % 右-口右竖
    [4.5 6.5], [7 7];   % 右-口下横
    [4 7], [6 6];       % 下-田上横
    [4 4], [4 6];       % 下-田左竖
    [7 7], [4 6];       % 下-田右竖
    [4 7], [4 4];       % 下-田下横
    [4 7], [5 5];       % 下-田中横
    [5.5 5.5], [4 6];   % 下-田中竖
};

% 初始化坐标存储矩阵 (Name: Fu_Stroke_Coordinates)
% 格式：[线段序号, X1, Y1, X2, Y2]
Fu_Stroke_Coordinates = zeros(length(strokes), 5);

fprintf('--- Fu_Stroke_Coordinates (缩小0.7倍后的坐标) ---\n');
fprintf('%-6s | %-8s | %-8s | %-8s | %-8s\n', '序号', 'X1', 'Y1', 'X2', 'Y2');
fprintf('------------------------------------------------------------\n');

for i = 1:length(strokes)
    origX = strokes{i, 1};
    origY = strokes{i, 2};
    
    % 计算映射后的坐标
    newX = finalX(origX, origY);
    newY = finalY(origX, origY);
    
    % 绘制
    plot(newX, newY, 'k', 'LineWidth', lw);
    
    % 存储坐标
    Fu_Stroke_Coordinates(i, :) = [i, newX(1), newY(1), newX(2), newY(2)];
    
    % 打印坐标
    fprintf('%-8d | %-8.3f | %-8.3f | %-8.3f | %-8.3f\n', ...
        Fu_Stroke_Coordinates(i,1), Fu_Stroke_Coordinates(i,2), ...
        Fu_Stroke_Coordinates(i,3), Fu_Stroke_Coordinates(i,4), ...
        Fu_Stroke_Coordinates(i,5));
end

title('旋转、映射且缩小0.7倍的"福"字');