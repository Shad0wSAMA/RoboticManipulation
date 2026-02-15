function IK_App

%% Robot Parameters
L1 = 0.077;
L2 = 0.130;
L3 = 0.124;
L4 = 0.126;

%% ===== USER INPUT BLOCK =====
x_target = input('Enter X (m): ');
y_target = input('Enter Y (m): ');
z_target = input('Enter Z (m): ');


%% ================= USER INPUT MOTION SIMULATION =================

disp('--- Manual Target Simulation ---');

x_target = input('Enter X (m): ');
y_target = input('Enter Y (m): ');
z_target = input('Enter Z (m): ');

% 初始位置（当前末端）
q_start = [0 0 0 0];

% 目标姿态
q_goal = ik_pos_vertical(x_target,y_target,z_target,L1,L2,L3,L4);

% 检查是否可达
if any(isnan(q_goal))
    error('Target not reachable');
end

% ===== cubic interpolation =====
tf = 2;      % total time
dt = 0.03;
tvec = 0:dt:tf;

for k = 1:length(tvec)
    
    t = tvec(k);
    s = 3*(t/tf)^2 - 2*(t/tf)^3;  % cubic scaling
    
    q = q_start + s*(q_goal - q_start);
    
    % FK
    [T_all,P_all] = fk_all(q,L1,L2,L3,L4);
    
    cla
    hold on
    grid on
    axis equal
    view(135,25)
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-0.3 0.4]);
    ylim([-0.3 0.3]);
    zlim([0 0.4]);
    
    % draw robot links
    for i = 1:size(P_all,2)-1
        line([P_all(1,i) P_all(1,i+1)], ...
             [P_all(2,i) P_all(2,i+1)], ...
             [P_all(3,i) P_all(3,i+1)], ...
             'LineWidth',5,'Color',[0.5 0.8 1]);
    end
    
    % draw joints
    for i = 1:size(P_all,2)
        draw_bubble(P_all(:,i),0.015,[0 0 0.8]);
    end
    
    % draw frames
    for i = 1:length(T_all)
        draw_frame(T_all{i},0.05);
    end
    
    drawnow
end


%% Create UI
fig = uifigure('Name','OpenManipulator IK App','Position',[500 300 400 400]);

% Input labels
uilabel(fig,'Text','X (m)','Position',[50 330 60 22]);
uilabel(fig,'Text','Y (m)','Position',[50 290 60 22]);
uilabel(fig,'Text','Z (m)','Position',[50 250 60 22]);

% Input fields
xField = uieditfield(fig,'numeric','Position',[120 330 100 22],'Value',0.18);
yField = uieditfield(fig,'numeric','Position',[120 290 100 22],'Value',0.00);
zField = uieditfield(fig,'numeric','Position',[120 250 100 22],'Value',0.12);

% Output labels
thetaLabel = uitextarea(fig, ...
    'Position',[40 50 320 140], ...
    'Editable','off', ...
    'FontSize',14);


% Compute button
uibutton(fig,'Text','Compute IK',...
    'Position',[50 200 120 30],...
    'ButtonPushedFcn',@computeIK);

% Send button (optional)
uibutton(fig,'Text','Send To Robot',...
    'Position',[200 200 150 30],...
    'ButtonPushedFcn',@sendToRobot);

%% Compute IK callback
    function computeIK(~,~)
        x = xField.Value;
        y = yField.Value;
        z = zField.Value;

        theta = myIK(x,y,z);

        if any(~isfinite(theta))
            thetaLabel.Value = "Unreachable position!";
        else
            thetaLabel.Value = sprintf([ ...
                'θ1 = %.3f rad (%.1f°)\n' ...
                'θ2 = %.3f rad (%.1f°)\n' ...
                'θ3 = %.3f rad (%.1f°)\n' ...
                'θ4 = %.3f rad (%.1f°)'], ...
                theta(1),rad2deg(theta(1)), ...
                theta(2),rad2deg(theta(2)), ...
                theta(3),rad2deg(theta(3)), ...
                theta(4),rad2deg(theta(4)));
        end
    end

%% Send to Robot (connect to your existing motor function)
    function sendToRobot(~,~)
        x = xField.Value;
        y = yField.Value;
        z = zField.Value;

        theta = myIK(x,y,z);

        if any(~isfinite(theta))
            uialert(fig,'Unreachable position','Error');
            return;
        end

        disp("Sending angles to robot:");
        disp(theta);

        % 👉 在这里调用你已有的发送函数
        % sendToMotors(theta);

    end

%% IK Function
    function theta = myIK(x, y, z)

        % Base yaw
        q1 = atan2(y,x);

        % Planar projection
        r  = hypot(x,y);
        zp = z - L1;

        % End-effector pitch
        phi = -pi/2;

        % Wrist center
        rw = r  - L4*cos(phi);
        zw = zp - L4*sin(phi);

        % 2R IK
        d2 = rw^2 + zw^2;
        c3 = (d2 - L2^2 - L3^2)/(2*L2*L3);

        if abs(c3) > 1
            theta = [NaN NaN NaN NaN];
            return;
        end

        s3 = -sqrt(1-c3^2);   % elbow-down
        q3 = atan2(s3,c3);

        alpha = atan2(zw,rw);
        beta  = atan2(L3*s3, L2 + L3*c3);
        q2 = alpha - beta;

        q4 = phi - q2 - q3;

        theta = [q1 q2 q3 q4];
    end

end
