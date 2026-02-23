clc, clear;
figure; hold on; axis equal; grid on;
% 目标坐标轴范围
xlim([0 9]); ylim([-9 9]);
lw = 6; 

% 这里的 finalX 和 finalY 采用了统一缩放系数 0.9
% 并且对 Y 坐标进行了 -4.5 的偏移以实现垂直居中
finalX = @(x, y) (-y + 10) * 0.9;
finalY = @(x, y) (x * 0.9) - 4.5;

%% ===== 左边：示 (旋转+等比例映射) =====
plot(finalX([2 3],[9 9]), finalY([2 3],[9 9]), 'k', 'LineWidth', lw)
plot(finalX([1.5 3.5],[8 8]), finalY([1.5 3.5],[8 8]), 'k', 'LineWidth', lw)
plot(finalX([2.5 2.5],[4 8]), finalY([2.5 2.5],[4 8]), 'k', 'LineWidth', lw)
plot(finalX([1.5 2],[5 7]), finalY([1.5 2],[5 7]), 'k', 'LineWidth', lw)
plot(finalX([3.5 3],[6 7]), finalY([3.5 3],[6 7]), 'k', 'LineWidth', lw)

%% ===== 右边：畐 (旋转+等比例映射) =====
plot(finalX([4.5 6.5],[9 9]), finalY([4.5 6.5],[9 9]), 'k', 'LineWidth', lw)
plot(finalX([4.5 6.5],[8 8]), finalY([4.5 6.5],[8 8]), 'k', 'LineWidth', lw)
plot(finalX([4.5 4.5],[7 8]), finalY([4.5 4.5],[7 8]), 'k', 'LineWidth', lw)
plot(finalX([6.5 6.5],[7 8]), finalY([6.5 6.5],[7 8]), 'k', 'LineWidth', lw)
plot(finalX([4.5 6.5],[7 7]), finalY([4.5 6.5],[7 7]), 'k', 'LineWidth', lw)

%% ===== 下部口结构 (旋转+等比例映射) =====
plot(finalX([4 7],[6 6]), finalY([4 7],[6 6]), 'k', 'LineWidth', lw)
plot(finalX([4 4],[4 6]), finalY([4 4],[4 6]), 'k', 'LineWidth', lw)
plot(finalX([7 7],[4 6]), finalY([7 7],[4 6]), 'k', 'LineWidth', lw)
plot(finalX([4 7],[4 4]), finalY([4 7],[4 4]), 'k', 'LineWidth', lw)
plot(finalX([4 7],[5 5]), finalY([4 7],[5 5]), 'k', 'LineWidth', lw)
plot(finalX([5.5 5.5],[4 6]), finalY([5.5 5.5],[4 6]), 'k', 'LineWidth', lw)

title('比例协调的旋转映射"福"字');