%% =================== DH 方形轨迹绘制 ===================
clear; clc; close all;

%% ---------- 1. 机械臂参数 ----------
L1 = 0.077;   % 基座高度
L2 = 0.130;
L3 = 0.124;
L4 = 0.126;    % 末端（姿态用，这里固定）

%% ---------- 2. DH Table 模板 ----------
% [a, alpha, d, theta]
DH0 = [ ...
    0   deg2rad(90)  L1  0;
    L2  0            0   0;
    L3  0            0   0;
    L4  0            0   0];

%% ---------- 3. XY 平面 10×10 方形（中心在原点） ----------
side = 0.1;
half = side/2;
N = 50;

z0 = 0.15;          % 固定高度（水平面）
x_offset = 0.1;    % 向前平移，避免 r=0 奇异

p1 = [ x_offset - half,  -half, z0];
p2 = [ x_offset + half,  -half, z0];
p3 = [ x_offset + half,   half, z0];
p4 = [ x_offset - half,   half, z0];

traj = [ ...
    interp(p1,p2,N);
    interp(p2,p3,N);
    interp(p3,p4,N);
    interp(p4,p1,N)];


%% ---------- 4. 绘图窗口 ----------
figure('Color','w'); hold on; grid on; axis equal;
xlabel X; ylabel Y; zlabel Z;
view(40,25);

R = L2 + L3 + L4;      % 约 0.38
margin = 0.05;

axis equal
xlim([-R-margin, R+margin])
ylim([-R-margin, R+margin])
zlim([0, R+margin])
view(40,25)

hLink  = plot3(nan,nan,nan,'LineWidth',3);
hTrace = plot3(nan,nan,nan,'r','LineWidth',2);

trace = [];

%% ---------- 5. 动画循环 ----------
for k = 1:size(traj,1)

    xd = traj(k,1);
    yd = traj(k,2);
    zd = traj(k,3);

    % ===== 逆运动学（位置）=====
    theta1 = atan2(yd, xd);
    r = sqrt(xd^2 + yd^2);
    z = zd - L1;
    
    D = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3);
    D = max(min(D,1),-1);   % 数值保护
    
    % 两个肘部解：+sqrt 和 -sqrt
    theta3_candidates = [ atan2( sqrt(1-D^2), D ), ...
                          atan2(-sqrt(1-D^2), D ) ];
    
    theta4 = 0;  % 姿态先固定
    clearance = 0.01;   % 离桌面最小安全高度（按你的单位改，桌面 z=0）
    
    best = [];
    bestScore = inf;
    
    for s = 1:2
        theta3 = theta3_candidates(s);
        theta2 = atan2(z, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3));
        theta4 = -pi/2-theta2-theta3;

        DH = DH0;
        DH(:,4) = [theta1; theta2; theta3; theta4];
    
        [~, P] = fkine_from_DH(DH);
    
        % P(1,:)是基座，P(2,:)关节1末端，P(3,:)关节2末端（常被当作“肘”附近），P(4,:)关节3末端
        z_elbow = P(3,3);      % 重点看“肘部”高度（你也可以改成 min(P(2:4,3))）
        z_min_links = min(P(2:4,3));  % 更保守：看所有中间关节最低点
    
        if z_min_links >= clearance
            % 评分：选“离桌子更远”的（你也可以选更平滑的那支）
            score = -z_elbow; % 越高越好 -> score 越小越差？这里用负号，等价选 z_elbow 最大
            if score < bestScore
                bestScore = score;
                best = struct('DH',DH,'P',P);
            end
        end
    end
    
    if isempty(best)
        % 两个解都不满足安全高度：最简单处理是抬高轨迹 or 跳过
        warning('点不可在当前安全高度下到达：k=%d', k);
        continue;
    end
    
    % 用选出来的安全解
    P = best.P;

    % 绘图
    set(hLink,'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));

    trace = [trace; P(end,:)];
    set(hTrace,'XData',trace(:,1),'YData',trace(:,2),'ZData',trace(:,3));

    drawnow;
    pause(0.03);
end

title('末端绘制 10×10 正方形');

%% =================== 函数区 ===================

function A = dh_std(a, alpha, d, theta)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ ct  -st*ca  st*sa  a*ct;
          st   ct*ca -ct*sa  a*st;
          0    sa     ca      d;
          0    0      0       1 ];
end

function [Tlist, P] = fkine_from_DH(DH)
    n = size(DH,1);
    T = eye(4);
    P = zeros(n+1,3);
    Tlist = cell(n,1);
    for i = 1:n
        A = dh_std(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
        T = T*A;
        Tlist{i} = T;
        P(i+1,:) = T(1:3,4)';
    end
end

function pts = interp(p1,p2,N)
    t = linspace(0,1,N)';
    pts = p1.*(1-t) + p2.*t;
end
