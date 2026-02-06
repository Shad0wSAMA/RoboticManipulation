close all; clear; clc;
theta0 = 0;
thetaf = 1.0;
vmax   = 1.0;
alpha  = 2.0;
dt = 1/30;

theta = linspace(theta0, thetaf, 1000);
v = zeros(size(theta));

for i = 2:length(theta)
    [v_cmd, v_des] = trapezoidVelLimited(theta(i), theta0, thetaf, vmax, alpha, alpha,dt,  v(i-1));
    v(i) = v_cmd;
end

plot(theta, v, 'LineWidth', 2);
xlabel('theta');
ylabel('velocity');
grid on;

function [v_cmd, v_des] = trapezoidVelLimited(theta, theta0, thetaf, vmax, alpha_plan, alpha_lim, dt, v_last)

    % -------- 参数保护 --------
    if dt <= 0 || vmax <= 0 || alpha_plan <= 0 || alpha_lim <= 0
        v_des = 0;
        v_cmd = 0;
        return;
    end

    % -------- 1) 先算理想速度 v_des（位置域梯形/三角）--------
    v_des = trapezoidVel(theta, theta0, thetaf, vmax, alpha_plan);

    % 数值安全：避免 NaN / Inf
    if ~isfinite(v_des)
        v_des = 0;
    end

    % 速度限幅（绝对最大速度）
    v_des = clamp(v_des, -vmax, vmax);

    % -------- 2) 加速度限幅（每周期速度变化限幅）--------
    dv_max = alpha_lim * dt;
    dv = v_des - v_last;
    dv = clamp(dv, -dv_max, dv_max);

    v_cmd = v_last + dv;

    % 最终再限一次速度（双保险）
    v_cmd = clamp(v_cmd, -vmax, vmax);
end

% ====== 内部小工具：限幅 ======
function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end



function v = trapezoidVel(theta, theta0, thetaf, vmax, alpha)
% trapezoidVel
% 位置域梯形/三角速度规划
%
% 输入：
%   theta   - 当前角度
%   theta0  - 起始角度
%   thetaf  - 目标角度
%   vmax    - 最大角速度 (>=0)
%   alpha   - 角加速度 (>=0)
%
% 输出：
%   v       - 速度指令（带方向）

    % -------- 基本保护 --------
    if alpha <= 0 || vmax <= 0
        v = 0;
        return;
    end

    % 运动方向
    dir = sign(thetaf - theta0);
    if dir == 0
        v = 0;
        return;
    end

    % 统一到“正方向运动”的坐标系
    s  = dir * (theta  - theta0);   % 已走距离
    sf = dir * (thetaf - theta0);   % 总距离

    % 越界保护
    if s <= 0
        v = 0;
        return;
    end
    if s >= sf
        v = 0;
        return;
    end

    % -------- 梯形关键参数 --------
    % 理论加速到 vmax 需要的距离
    s_acc = vmax^2 / (2 * alpha);

    % -------- 三段速度计算 --------
    if sf >= 2 * s_acc
        % ===== 梯形速度 =====
        if s < s_acc
            % 加速段
            v_mag = sqrt(2 * alpha * s);
        elseif s > sf - s_acc
            % 减速段
            v_mag = sqrt(2 * alpha * (sf - s));
        else
            % 匀速段
            v_mag = vmax;
        end
    else
        % ===== 三角速度（达不到 vmax）=====
        v_peak = sqrt(alpha * sf);
        s_mid  = sf / 2;

        if s < s_mid
            v_mag = sqrt(2 * alpha * s);
        else
            v_mag = sqrt(2 * alpha * (sf - s));
        end

        % 限制峰值（数值安全）
        v_mag = min(v_mag, v_peak);
    end

    % 恢复方向
    v = dir * v_mag;
end
