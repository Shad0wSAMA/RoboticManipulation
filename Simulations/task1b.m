%% DH 动态建模演示（标准DH）
% 说明：
% - 用一个 DH table（a, alpha, d, theta）描述机构
% - 每一帧用 FK(正运动学)算出各关节坐标系与关节点位置
% - 3D 动画展示连杆与关节坐标轴

clear; clc; close all;

%% ========== 1) 定义连杆长度（按需修改） ==========
L1 = 0.077;   % base 到关节2 的高度
L2 = 0.130;
L3 = 0.124;
L4 = 0.126;

%% ========== 2) 定义 DH Table（标准DH） ==========
% DH 表格式：[a_i, alpha_i, d_i, theta_i]
% 其中 theta_i 是关节变量（转角），动画时会更新该列
DH0 = [ ...
    0,   deg2rad(90),  L1,  0;   % i=1
    L2,  0,            0,   0;   % i=2
    L3,  0,            0,   0;   % i=3
    L4,  0,            0,   0];  % i=4

n = size(DH0,1);

%% ========== 3) 画图窗口设置 ==========
figure('Color','w'); hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(35,25);

% 坐标范围（简单给大一点，避免跑出画面）
R = (L1+L2+L3+L4)*1.2;
xlim([-R R]); ylim([-R R]); zlim([0 R]);

% 绘制基座坐标系
drawFrame(eye(4), 0.10, '0');

% 用于更新的图形句柄（连杆、关节点、坐标系）
hLink  = plot3(nan,nan,nan,'LineWidth',3);          % 连杆折线
hJoint = plot3(nan,nan,nan,'o','MarkerSize',7, ...
               'MarkerFaceColor','k','MarkerEdgeColor','k'); % 关节点

% 关节坐标系（每个关节画3根轴，方便看姿态）
hFrames = gobjects(n,1);

%% ========== 4) 动画参数 ==========
Tend = 10;           % 动画总时长（秒）
fps  = 30;
dt   = 1/fps;
tvec = 0:dt:Tend;

% 关节角运动（示例：正弦摆动，可改成你的轨迹/插值）
% 注意：这里默认 4 个关节都是转动关节
for k = 1:length(tvec)
    t = tvec(k);

    theta1 = deg2rad( 60*sin(0.6*t) );
    theta2 = deg2rad( 45*sin(0.9*t + 0.5)+90);
    theta3 = deg2rad( 40*sin(0.7*t + 1.0));
    theta4 = deg2rad( 90*sin(t) );
    
    DH = DH0;
    DH(:,4) = [theta1; theta2; theta3; theta4];  % 更新 theta 列

    % 正运动学：算每一节的齐次变换
    [Tlist, P] = fkine_from_DH(DH);

    % P: (n+1) x 3，包含基座点P0 + 各关节点位置 P1..Pn
    % 更新连杆与关节点
    set(hLink,  'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
    set(hJoint, 'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));

    % 更新关节坐标系（画在每个关节处：Tlist{i} 对应第 i 节末端坐标系）
    % 先删除旧的再画新的（简单粗暴但清晰）
    delete(hFrames(ishandle(hFrames)));
    for i = 1:n
        hFrames(i) = drawFrame(Tlist{i}, 0.08, sprintf('%d',i));
    end

    title(sprintf('DH 动画演示   t = %.2f s', t));
    drawnow;
end

%% ====================== 本脚本用到的函数 ======================

function A = dh_std(a, alpha, d, theta)
% 标准DH（Craig 常用定义）:
% A = RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
end

function [Tlist, P] = fkine_from_DH(DH)
% 输入 DH: n x 4 [a alpha d theta]
% 输出：
% - Tlist{i}: 从基座到第 i 节末端的齐次变换
% - P: (n+1) x 3，基座点 + 每个关节位置
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
% 在齐次变换 T 指定的位置/姿态画一个坐标系
% s: 轴长度
% tag: 标记文本
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
