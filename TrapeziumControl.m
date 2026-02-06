clc; clear; close all;

%% ===================== 机械臂结构定义 =====================
% 4轴：q1 底座yaw(绕z)；q2 q3 q4 pitch(都绕y，彼此平行)
% 4连杆长度（单位：m）
L = [0.30, 0.25, 0.20, 0.15];  % Link1..Link4

% 起点/终点关节角（单位：rad）
q0 = deg2rad([  0,  0, 0,  0]);     % [q1 q2 q3 q4]
qf = deg2rad([ 90,  90,  90, 90]);

% 每轴限制（你可以按真实舵机改）
wmax = deg2rad([120, 140, 160, 200]);   % rad/s
amax = deg2rad([600, 700, 800, 900]);   % rad/s^2

dt   = 0.005;   % 控制周期（动画建议 0.005~0.02）
tMax = 5.0;     % 仿真最大时长

%% ===================== 1) 多轴同步：算统一到位时间 T =====================
dq = abs(qf - q0);

Tmin = zeros(1,4);
for i = 1:4
    Tmin(i) = trapezoid_min_time(dq(i), wmax(i), amax(i));
end
T = max(Tmin);

% 若你想留裕量（更柔）：T = 1.2*T;
T = min(T, tMax);  % 防止超过最大仿真时长（可按需删）

fprintf('Per-joint Tmin (s): [%.3f %.3f %.3f %.3f]\n', Tmin);
fprintf('Synchronized T (s): %.3f\n', T);

%% ===================== 2) 生成同步轨迹（quintic） =====================
t = 0:dt:T;
N = numel(t);

q   = zeros(N,4);
qd  = zeros(N,4);
qdd = zeros(N,4);

for k = 1:N
    tau = t(k)/T; % 0..1
    [s, sd, sdd] = quintic_blend(tau);

    q(k,:)   = q0 + (qf - q0) * s;
    qd(k,:)  = (qf - q0) * (sd / T);
    qdd(k,:) = (qf - q0) * (sdd / (T^2));
end

%% ===================== 3) Forward Kinematics & 动画 =====================
% 预计算关键点位置（base + 每个关节/连杆端点）
P = zeros(N, 3, 5); % 5个点：P0 base, P1..P4 逐段末端（4连杆末端=末端执行器）
for k = 1:N
    P(k,:,:) = fk_points(q(k,:), L);
end

% 动画慢放（解决“动太快看不清”）
time_scale = 8;  % 越大越慢（比如 5~20）

%% 画布布局
fig = figure('Color','w','Name','4-DOF FK Sync Demo');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% 3D动画
ax3 = nexttile([2,1]);
grid(ax3,'on'); axis(ax3,'equal'); view(ax3, 35, 22);
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
title(ax3,'4-DOF Arm Animation (FK)');

% 坐标范围估计（根据长度）
reach = sum(L);
axis(ax3, [-reach reach -reach reach -reach reach]*1.1);
hold(ax3,'on');
plot3(ax3, 0,0,0,'k.','MarkerSize',18);

armLine = plot3(ax3, nan, nan, nan, 'LineWidth', 4);
jointPts = plot3(ax3, nan, nan, nan, 'o', 'MarkerSize', 6, 'LineWidth', 1.5);
eePath = plot3(ax3, nan, nan, nan, '-', 'LineWidth', 1.2); % 末端轨迹

% 右上：角度
axQ = nexttile;
grid(axQ,'on'); hold(axQ,'on');
title(axQ,'Joint Angles q(t)');
xlabel(axQ,'t (s)'); ylabel(axQ,'rad');
qLines = gobjects(1,4);
for i=1:4
    qLines(i) = animatedline(axQ, 'LineWidth', 1.2);
end
legend(axQ, {'q1 yaw','q2 pitch','q3 pitch','q4 pitch'}, 'Location','best');

% 右下：速度/加速度（用两条轴显示更清楚）
axW = nexttile;
grid(axW,'on'); hold(axW,'on');
title(axW,'Joint Velocities qd(t) and Accelerations qdd(t)');
xlabel(axW,'t (s)'); ylabel(axW,'qd (rad/s)');

wLines = gobjects(1,4);
for i=1:4
    wLines(i) = animatedline(axW, 'LineWidth', 1.2);
end
legend(axW, {'qd1','qd2','qd3','qd4'}, 'Location','best');

% 第二y轴画加速度
axA = axes('Position', axW.Position, 'Color','none', 'YAxisLocation','right', ...
    'XAxisLocation','bottom', 'XLim', axW.XLim, 'YLimMode','auto');
ylabel(axA,'qdd (rad/s^2)'); hold(axA,'on'); grid(axA,'off');
aLines = gobjects(1,4);
for i=1:4
    aLines(i) = animatedline(axA, 'LineStyle','--', 'LineWidth', 1.0);
end

% 末端轨迹缓存
eeX = zeros(N,1); eeY = zeros(N,1); eeZ = zeros(N,1);

%% 动画循环
for k = 1:N
    pts = squeeze(P(k,:,:));  % 3x5
    X = pts(1,:); Y = pts(2,:); Z = pts(3,:);

    % 更新机械臂线段和关节点
    set(armLine,  'XData', X, 'YData', Y, 'ZData', Z);
    set(jointPts, 'XData', X, 'YData', Y, 'ZData', Z);

    % 末端轨迹
    eeX(k) = X(end); eeY(k) = Y(end); eeZ(k) = Z(end);
    set(eePath, 'XData', eeX(1:k), 'YData', eeY(1:k), 'ZData', eeZ(1:k));

    % 右侧曲线实时更新
    for i=1:4
        addpoints(qLines(i), t(k), q(k,i));
        addpoints(wLines(i), t(k), qd(k,i));
        addpoints(aLines(i), t(k), qdd(k,i));
    end

    drawnow limitrate;
    pause(dt * time_scale);
end

%% ===================== 辅助函数 =====================
function Tmin = trapezoid_min_time(dtheta, wmax, amax)
% 按梯形/三角速度求最短到位时间（位移 dtheta >= 0）
    if dtheta <= 0
        Tmin = 0;
        return;
    end

    s_ad = (wmax^2) / amax; % 加速+减速所需位移
    if dtheta >= s_ad
        ta = wmax / amax;
        tc = (dtheta - s_ad) / wmax;
        Tmin = 2*ta + tc;
    else
        % 三角形：达不到 wmax
        Tmin = 2*sqrt(dtheta/amax);
    end
end

function [s, sd, sdd] = quintic_blend(tau)
% 5次多项式混合函数：tau in [0,1]
% s(0)=0,s(1)=1; sd(0)=sd(1)=0; sdd(0)=sdd(1)=0
    tau = max(0, min(1, tau));
    s   = 10*tau^3 - 15*tau^4 +  6*tau^5;
    sd  = 30*tau^2 - 60*tau^3 + 30*tau^4;
    sdd = 60*tau   -180*tau^2 +120*tau^3;
end

function P = fk_points(q, L)
% 4-DOF FK
% Joint1 axis ⟂ Joint2-4 axes
% Here: J1 about Z, J2-4 about Y (all parallel)
% Links extend along +X
% P: 3x5 points: base, after L1, after L2, after L3, after L4(end)

    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    p0 = [0;0;0;1];

    T = eye(4);
    P = zeros(3,5);

    % base
    tmp = T*p0; P(:,1)=tmp(1:3);

    % Joint1: about Z (perpendicular to Y)
    T = T * Rz(q1);

    % Joint2: about Y + Link1
    T = T * Ry(q2);
    T = T * Tx(L(1));
    tmp = T*p0; P(:,2)=tmp(1:3);

    % Joint3: about Y + Link2
    T = T * Ry(q3);
    T = T * Tx(L(2));
    tmp = T*p0; P(:,3)=tmp(1:3);

    % Joint4: about Y + Link3
    T = T * Ry(q4);
    T = T * Tx(L(3));
    tmp = T*p0; P(:,4)=tmp(1:3);

    % End-effector Link4 (no joint)
    T = T * Tx(L(4));
    tmp = T*p0; P(:,5)=tmp(1:3);
end

function T = Tx(a)
    T = [1 0 0 a;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
end

function T = Ry(th)
    c=cos(th); s=sin(th);
    T = [ c 0 s 0;
          0 1 0 0;
         -s 0 c 0;
          0 0 0 1];
end

function T = Rz(th)
    c=cos(th); s=sin(th);
    T = [ c -s 0 0;
          s  c 0 0;
          0  0 1 0;
          0  0 0 1];
end



