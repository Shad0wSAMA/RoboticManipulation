function dh_slider_demo()
%% DH 动态建模演示（标准DH）- 滑动条手动控制版（修复 end 报错）
% 保存为 dh_slider_demo.m，然后运行 dh_slider_demo

clearvars; clc; close all;

%% ========== 1) 定义连杆长度（按需修改） ==========
L1 = 0.077;
L2 = 0.130;
L3 = 0.124;
L4 = 0.126;

%% ========== 2) 定义 DH Table（标准DH） ==========
DH0 = [ ...
    0,   deg2rad(90),  L1,  0;   % i=1
    L2,  0,            0,   0;   % i=2
    L3,  0,            0,   0;   % i=3
    L4,  0,            0,   0];  % i=4
n = size(DH0,1);

%% ========== 3) 画图窗口设置 ==========
fig = figure('Color','w','Name','DH Slider Demo','NumberTitle','off');
axes('Parent',fig);
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(35,25);

R = (L1+L2+L3+L4)*1.2;
xlim([-R R]); ylim([-R R]); zlim([0 R]);

% 绘制基座坐标系
drawFrame(eye(4), 0.10, '0');

% 用于更新的图形句柄
hLink  = plot3(nan,nan,nan,'LineWidth',3);
hJoint = plot3(nan,nan,nan,'o','MarkerSize',7, ...
               'MarkerFaceColor','k','MarkerEdgeColor','k');
hFrames = gobjects(n,1);

%% ========== 4) UI：4个滑动条 ==========
set(fig,'Units','normalized');
set(gca,'Position',[0.05 0.28 0.90 0.70]); % 给底部留UI空间

degMin  = [-180 -180 -180 -180];
degMax  = [ 180  180  180  180];
degInit = [0 90 0 0];  % 初值（你原来 theta2 有 +90 偏置，这里直接设 90）

slider = gobjects(4,1);
valTxt = gobjects(4,1);

for i = 1:4
    y = 0.18 - (i-1)*0.05;

    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.05 y 0.08 0.04], ...
        'String',sprintf('th_%d',i), ...
        'BackgroundColor','w','HorizontalAlignment','left','FontSize',11);

    slider(i) = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.13 y+0.005 0.70 0.03], ...
        'Min',degMin(i),'Max',degMax(i),'Value',degInit(i), ...
        'Callback',@onSlider, 'Interruptible','off', 'BusyAction','cancel');

    valTxt(i) = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.85 y 0.10 0.04], ...
        'String',sprintf('%7.2f°',degInit(i)), ...
        'BackgroundColor','w','HorizontalAlignment','right','FontSize',11);
end

uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.05 0.02 0.12 0.05], ...
    'String','Reset', 'Callback',@onReset);

% 首次刷新
updatePlot();

%% ====================== 回调（嵌套函数） ======================
function onSlider(~,~)
    updatePlot();
end

function onReset(~,~)
    for ii = 1:4
        slider(ii).Value = degInit(ii);
    end
    updatePlot();
end

function updatePlot()
    % 读取滑动条角度（deg）
    deg = zeros(4,1);
    for ii = 1:4
        deg(ii) = slider(ii).Value;
        valTxt(ii).String = sprintf('%7.2f°',deg(ii));
    end

    % 更新 DH
    DH = DH0;
    DH(:,4) = deg2rad(deg);

    % FK
    [Tlist, P] = fkine_from_DH(DH);

    % 更新连杆与关节
    set(hLink,  'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
    set(hJoint, 'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));

    % 更新坐标系
    delete(hFrames(ishandle(hFrames)));
    for jj = 1:n
        hFrames(jj) = drawFrame(Tlist{jj}, 0.08, sprintf('%d',jj));
    end

    title(sprintf('DH 手动控制  \\theta=[%.1f, %.1f, %.1f, %.1f] deg',deg));
    drawnow limitrate;
end

end % <-- 主函数结束

%% ====================== 局部函数（不嵌套） ======================
function A = dh_std(a, alpha, d, theta)
% 标准DH（Craig）:
% A = RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
end

function [Tlist, P] = fkine_from_DH(DH)
    n = size(DH,1);
    T = eye(4);

    Tlist = cell(n,1);
    P = zeros(n+1,3);
    P(1,:) = [0 0 0];

    for i = 1:n
        a     = DH(i,1);
        alpha = DH(i,2);
        d     = DH(i,3);
        theta = DH(i,4);

        A = dh_std(a, alpha, d, theta);
        T = T * A;

        Tlist{i} = T;
        P(i+1,:) = T(1:3,4).';
    end
end

function h = drawFrame(T, s, tag)
    p = T(1:3,4);
    x = p + s*T(1:3,1);
    y = p + s*T(1:3,2);
    z = p + s*T(1:3,3);

    hx = plot3([p(1) x(1)], [p(2) x(2)], [p(3) x(3)], 'r', 'LineWidth',2);
    hy = plot3([p(1) y(1)], [p(2) y(2)], [p(3) y(3)], 'g', 'LineWidth',2);
    hz = plot3([p(1) z(1)], [p(2) z(2)], [p(3) z(3)], 'b', 'LineWidth',2);
    ht = text(p(1),p(2),p(3), ['  ',tag], 'FontSize',10, 'Color',[0 0 0]);

    h = hggroup;
    set([hx,hy,hz,ht],'Parent',h);
end
