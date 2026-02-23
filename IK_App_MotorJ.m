function IK_App_MotorJ

clc;
clear all;

%% Library
addpath(genpath('D:\EEE\Robotic Manipulation\LAB\DynamixelSDK-4.0.2\DynamixelSDK-4.0.2\matlab'));

lib_path = 'D:\EEE\Robotic Manipulation\LAB\DynamixelSDK-4.0.2\DynamixelSDK-4.0.2\c\build\win64\output\dxl_x64_c.dll';

header_path = 'D:\EEE\Robotic Manipulation\LAB\DynamixelSDK-4.0.2\DynamixelSDK-4.0.2\c\include\dynamixel_sdk\dynamixel_sdk.h';

if libisloaded('dxl_x64_c')
    unloadlibrary('dxl_x64_c');
end

[notfound, warnings] = loadlibrary( ...
    lib_path, ...
    header_path, ...
    'addheader','D:\EEE\Robotic Manipulation\LAB\DynamixelSDK-4.0.2\DynamixelSDK-4.0.2\c\include\dynamixel_sdk\port_handler.h', ...
    'addheader','D:\EEE\Robotic Manipulation\LAB\DynamixelSDK-4.0.2\DynamixelSDK-4.0.2\c\include\dynamixel_sdk\packet_handler.h');

loadlibrary(lib_path, header_path);

%% ================= Motor Parameters =================
clear global

global is_estop
is_estop = false;
global port_num PROTOCOL_VERSION
global DXL_ID1 DXL_ID2 DXL_ID3 DXL_ID4 DXL_GRIP
global ADDR_PRO_GOAL_POSITION ADDR_PRO_PRESENT_POSITION
global ADDR_PRO_TORQUE_ENABLE ADDR_PRO_OPERATING_MODE
global ADDR_PRO_PROFILE_VELOCITY ADDR_PRO_PROFILE_ACCELERATION
global angle_offset
angle_offset = [0 pi/2 -pi/2 0];
angle_offset = angle_offset(:);   % 强制变成列向量

DXL_ID1 = 11;
DXL_ID2 = 12;
DXL_ID3 = 13;
DXL_ID4 = 14;
DXL_GRIP = 15;

PROTOCOL_VERSION = 2.0;

% ---- Control Table Addresses ----
ADDR_PRO_TORQUE_ENABLE       = 64;
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_PROFILE_VELOCITY    = 112;
ADDR_PRO_PROFILE_ACCELERATION= 108;

BAUDRATE = 1000000;
DEVICENAME = 'COM3';

%% ================= Port Setup =================
port_num = portHandler(DEVICENAME);
packetHandler();

if ~openPort(port_num)
    error('Failed to open port');
end

if ~setBaudRate(port_num, BAUDRATE)
    error('Failed to set baudrate');
end

fprintf("Port Opened & Baudrate Set\n");

%% ================= Motor Setup =================

DXL_IDS = [DXL_ID1 DXL_ID2 DXL_ID3 DXL_ID4 DXL_GRIP];

PROFILE_VEL = 100;      % <-- 统一速度
PROFILE_ACC = 20;       % <-- 统一加速度

for id = DXL_IDS

    % Position Mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_OPERATING_MODE, 3);

    % Set velocity profile
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ...
        ADDR_PRO_PROFILE_VELOCITY, typecast(int32(PROFILE_VEL),'uint32'));

    % Set acceleration profile
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ...
        ADDR_PRO_PROFILE_ACCELERATION, typecast(int32(PROFILE_ACC),'uint32'));

    % Enable torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, 1);

end

fprintf("All motors initialized in Position Mode\n");

%% Robot Parameters
L1 = 0.077;
L2 = 0.130;
L3 = 0.124;
L4 = 0.145;
gripper_closed = false;
scaleXY = 0.2 / 7.5;

q_home = [0;
          2.059;
         -1.313;
         -2.317];


%% Home Position
x_home = 0.03;
y_home = 0.00;
z_home = 0.15;

%% current angle

% handle for previous path line
pathHandle = [];

% ===== Read real motor position as initial state =====

sendJointToMotor([0 0 0 0]);

current_q = readJointFromMotor();
current_q = current_q(:);

%% Create Main UI Window
fig = uifigure('Name','OpenManipulator IK Control',...
               'Position',[300 200 900 500]);

fig.CloseRequestFcn = @onClose;

%% Create 3D Axes inside UI
ax = uiaxes(fig,'Position',[300 -20 550 400]);
axis(ax,'equal')
grid(ax,'on')
view(ax,135,25)
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
xlim(ax,[-0.5 0.5]); ylim(ax,[-0.5 0.5]); zlim(ax,[0 0.5]);
hold(ax,'on')

%% ================= MOVE PANEL =================
movePanel = uipanel(fig,...
    'Title','MOVE',...
    'Position',[30 360 250 100]);

uilabel(movePanel,'Text','X','Position',[10 40 20 22]);
uilabel(movePanel,'Text','Y','Position',[90 40 20 22]);
uilabel(movePanel,'Text','Z','Position',[170 40 20 22]);

moveX = uieditfield(movePanel,'numeric',...
    'Position',[10 15 60 22],'Value',0.18);

moveY = uieditfield(movePanel,'numeric',...
    'Position',[90 15 60 22],'Value',0.00);

moveZ = uieditfield(movePanel,'numeric',...
    'Position',[170 15 60 22],'Value',0.12);

%% ================= GRAB PANEL =================
grabPanel = uipanel(fig,...
    'Title','GRAB',...
    'Position',[300 360 250 100]);

uilabel(grabPanel,'Text','X','Position',[10 40 20 22]);
uilabel(grabPanel,'Text','Y','Position',[90 40 20 22]);
uilabel(grabPanel,'Text','Z','Position',[170 40 20 22]);

grabX = uieditfield(grabPanel,'numeric',...
    'Position',[10 15 60 22],'Value',0.20);

grabY = uieditfield(grabPanel,'numeric',...
    'Position',[90 15 60 22],'Value',0.00);

grabZ = uieditfield(grabPanel,'numeric',...
    'Position',[170 15 60 22],'Value',0.05);

%% ================= PUT PANEL =================
putPanel = uipanel(fig,...
    'Title','PUT',...
    'Position',[570 360 250 100]);

uilabel(putPanel,'Text','X','Position',[10 40 20 22]);
uilabel(putPanel,'Text','Y','Position',[90 40 20 22]);
uilabel(putPanel,'Text','Z','Position',[170 40 20 22]);

putX = uieditfield(putPanel,'numeric',...
    'Position',[10 15 60 22],'Value',-0.20);

putY = uieditfield(putPanel,'numeric',...
    'Position',[90 15 60 22],'Value',0.00);

putZ = uieditfield(putPanel,'numeric',...
    'Position',[170 15 60 22],'Value',0.05);


%% Output Display
thetaLabel = uitextarea(fig,...
    'Position',[40 50 200 140],...
    'Editable','off',...
    'FontSize',13);

%% Buttons
uibutton(fig,'Text','Move Robot',...
    'Position',[10 270 120 35],...
    'ButtonPushedFcn',@moveRobot);

uibutton(fig,'Text','Return Home',...
    'Position',[150 270 120 35],...
    'ButtonPushedFcn',@goHome);

uibutton(fig,'Text','Grab Demo',...
    'Position',[10 220 120 35],...
    'ButtonPushedFcn',@grabDemo);

uibutton(fig,'Text','E-STOP',...
    'Position',[10 170 120 35],...
    'BackgroundColor',[1 0.3 0.3],...
    'ButtonPushedFcn',@estop);

%% ================= Draw Initial Robot =================
drawInitialRobot();

%% Grab
    function grabDemo(~,~)
        x_grab = grabX.Value * scaleXY;
        y_grab = grabY.Value * scaleXY;
        z_grab = grabZ.Value;

        x_put = putX.Value * scaleXY;
        y_put = putY.Value * scaleXY;
        z_put = putZ.Value;
    
        grabObject(x_grab,y_grab,z_grab, x_put, y_put, z_put);
    end


%% Callback: Move Robot
    function moveRobot(~,~)
    
        x = moveX.Value * scaleXY;
        y = moveY.Value * scaleXY;
        z = moveZ.Value;
    
        moveToTarget(x,y,z);
    end


%% MovetoTarget
function moveToTarget(x,y,z)
    current_q = current_q(:);

    if is_estop
        return;
    end

    % Remove previous path
    if ~isempty(pathHandle) && isvalid(pathHandle)
        delete(pathHandle);
        pathHandle = [];
    end

    % Draw current pose once before motion starts
    [T_all, P_all] = fk_all(current_q, L1, L2, L3, L4);
    toolPos = P_all(:,end);
    
    cla(ax); hold(ax,'on')
    for i = 1:size(P_all,2)-1
        plot3(ax,[P_all(1,i) P_all(1,i+1)],...
                 [P_all(2,i) P_all(2,i+1)],...
                 [P_all(3,i) P_all(3,i+1)],...
                 'LineWidth',5,'Color',[0.5 0.8 1]);
    end
    for i = 1:size(P_all,2)
        draw_bubble_ui(ax,P_all(:,i),0.015,[0 0 0.8]);
    end
    for i = 1:numel(T_all)
        draw_frame_ui(ax,T_all{i},0.05);
    end
    draw_gripper(ax, toolPos, 0.02, gripper_closed);
    drawnow


    % Initial end-effector position
    [T_all, P_all] = fk_all(current_q, L1, L2, L3, L4);
    p0 = P_all(:, end);
    pf = [x; y; z];

    % Check reachability using IK once (only for validation)
    [q_goal, phi_used] = ik_search_phi(x,y,z,L1,L2,L3,L4, current_q);
    if any(isnan(q_goal))
        thetaLabel.Value = "Unreachable Position!";
        return;
    end

    % Display goal angles (informational only)
    thetaLabel.Value = sprintf([ ...
        'θ1 = %.3f rad (%.1f°)\n' ...
        'θ2 = %.3f rad (%.1f°)\n' ...
        'θ3 = %.3f rad (%.1f°)\n' ...
        'θ4 = %.3f rad (%.1f°)\n' ...
        '\nφ selected = %.3f rad (%.1f°)'], ...
        q_goal(1),rad2deg(q_goal(1)), ...
        q_goal(2),rad2deg(q_goal(2)), ...
        q_goal(3),rad2deg(q_goal(3)), ...
        q_goal(4),rad2deg(q_goal(4)), ...
        phi_used,rad2deg(phi_used));

    % Trajectory timing
    tf = 1.0;        % total motion time (seconds)
    dt = 0.03;       % control step
    tvec = 0:dt:tf;

    % Store trajectory
    pathPoints = zeros(3, numel(tvec));

    % Keep initial elbow posture as reference
    q_reference = current_q(:);
    
    for k = 1:numel(tvec)
        k_end = 0;
        if is_estop
            return;
        end

        t = tvec(k);
        tau = t / tf;

        q_real = readJointFromMotor();
        current_q = q_real;

        % Cubic time scaling
        s = 3*tau^2 - 2*tau^3;

        % Desired Cartesian position along straight line
        p_des = p0 + s*(pf - p0);

        % Current forward kinematics
        [T_all, P_all] = fk_all(q_real, L1, L2, L3, L4);
        p_current = P_all(:,end);

        % Position error
        e = p_des - p_current;

        % Stop only when near the final time AND close to the final goal
        e_goal = pf - p_current;
        if (tau > 0.98) && (norm(e_goal) < 0.002)
            break;
        end

        % Cartesian velocity command (proportional control)
        Kp = 4;
        pdot = Kp * e;

        % Compute Jacobian
        Jv = jacobian_from_fk(T_all);

        % Damped Least Squares inverse
        JJt = Jv*Jv';
        sigma = sqrt(det(JJt));
        
        if sigma < 0.01
            lambda = 0.2;
        else
            lambda = 0.05;
        end

        A = Jv*Jv' + lambda^2 * eye(3);
        J_pinv = Jv' / A;

        % ===== Primary task =====
        qdot_task = J_pinv * pdot;      % 4×1
        
        % ===== Secondary task =====
        k_null = 1.5;
        
        current_q = current_q(:);      % 强制列
        q_reference = q_reference(:);  % 强制列
        
        qdot_null = -k_null * (current_q - q_reference); % 4×1
        
        % ===== Null-space projection =====
        N = eye(4) - J_pinv * Jv;       % 4×4
        
        qdot = qdot_task + N * qdot_null; % 4×1


        % Optional joint velocity limit
        max_speed = 2;  % rad/s
        qdot = max(min(qdot, max_speed), -max_speed);

        % Integrate joint velocities
        q_real = readJointFromMotor();

        q_cmd = q_real + qdot * dt;

        q_min = [-2.8; -2.0; -2.0; -2.0];
        q_max = [ 2.8;  2.0;  2.0;  2.0];
                
        q_cmd = max(min(q_cmd, q_max), q_min);

        sendJointToMotor(q_cmd);

        current_q = q_cmd;

        % Recompute FK for drawing
        [T_all, P_all] = fk_all(current_q, L1, L2, L3, L4);
        toolPos = P_all(:,end);

        pathPoints(:,k) = toolPos;

        % ================= Drawing =================
        cla(ax); hold(ax,'on')

        % Draw links
        for i = 1:size(P_all,2)-1
            plot3(ax,...
                [P_all(1,i) P_all(1,i+1)],...
                [P_all(2,i) P_all(2,i+1)],...
                [P_all(3,i) P_all(3,i+1)],...
                'LineWidth',5,'Color',[0.5 0.8 1]);
        end

        % Draw joints
        for i = 1:size(P_all,2)
            draw_bubble_ui(ax,P_all(:,i),0.015,[0 0 0.8]);
        end

        % Draw frames
        for i = 1:numel(T_all)
            draw_frame_ui(ax,T_all{i},0.05);
        end

        % Draw gripper
        if gripper_closed
            draw_gripper(ax, toolPos, 0.02, true);
        else
            draw_gripper(ax, toolPos, 0.02, false);
        end

        drawnow
        k_end = k;
    end

    % Draw final path
    pathHandle = plot3(ax,...
        pathPoints(1,1:k_end), pathPoints(2,1:k_end), pathPoints(3,1:k_end),...
        'k--','LineWidth',1.5);

end

%% Callback: Return Home
    function goHome(~,~)
        moveToJointTarget(q_home);
    end

%% Grab Object
    function grabObject(x_obj,y_obj,z_obj,x_target,y_target,z_target)
    
        z_safe1 = z_obj + 0.05;
        z_safe2 = z_target + 0.05;

        % retreat
        moveToJointTarget(q_home);
    
        % approach
        moveToTarget(x_obj,y_obj,z_safe1);
    
        % descend
        moveToTarget(x_obj,y_obj,z_obj);
    
        % close gripper
        gripper_closed = true;
        setGripper(true);
    
        pause(0.5)
    
        % lift
        moveToTarget(x_obj,y_obj,z_safe1);
    
        % move to target
        moveToTarget(x_target,y_target,z_safe2);
    
        % descend
        moveToTarget(x_target,y_target,z_target);
    
        % open
        gripper_closed = false;
        setGripper(false);
    
        pause(0.5)
    
        % retreat
        moveToJointTarget(q_home);
    
    end

%% Move to Home at Startup
moveX.Value = x_home/scaleXY;
moveY.Value = y_home/scaleXY;
moveZ.Value = z_home;
%moveToTarget(x_home,y_home,z_home);

%% ===================== FK =====================
    function [T_all, P_all] = fk_all(q,L1,L2,L3,L4)
        DH = [ ...
            0   pi/2   L1   q(1);
            L2  0      0    q(2);
            L3  0      0    q(3);
            L4  0      0    q(4)];

        T = eye(4);
        T_all = cell(1,4);
        P_all = [0;0;0];

        for i = 1:4
            a = DH(i,1);
            alpha = DH(i,2);
            d = DH(i,3);
            th = DH(i,4);

            A = [cos(th) -sin(th)*cos(alpha)  sin(th)*sin(alpha)  a*cos(th);
                 sin(th)  cos(th)*cos(alpha) -cos(th)*sin(alpha)  a*sin(th);
                 0        sin(alpha)          cos(alpha)          d;
                 0        0                   0                   1];

            T = T*A;
            T_all{i} = T;
            P_all(:,end+1) = T(1:3,4);
        end
    end

%% ===================== IK =====================
    function q = ik_pos_with_phi(x,y,z,L1,L2,L3,L4,phi)
        q1 = atan2(y,x);
        r  = hypot(x,y);
        zp = z - L1;

        rw = r  - L4*cos(phi);
        zw = zp - L4*sin(phi);

        d2 = rw^2 + zw^2;
        c3 = (d2 - L2^2 - L3^2)/(2*L2*L3);

        if abs(c3) > 1
            q = [NaN NaN NaN NaN];
            return;
        end

        s3 = -sqrt(1-c3^2);
        q3 = atan2(s3,c3);

        alpha = atan2(zw,rw);
        beta  = atan2(L3*s3, L2 + L3*c3);
        q2 = alpha - beta;

        q4 = phi - q2 - q3;

        q = [q1 q2 q3 q4];
    end

%% Search available phi
function [q, phi_used] = ik_search_phi(x,y,z,L1,L2,L3,L4, q_prev)

    % reject
%    r = hypot(x,y);
%    if r < 0.02
%        q = [NaN NaN NaN NaN];
%        phi_used = NaN;
%        return
%    end

    % initial q
    if nargin < 8
        q_prev = [];
    end

    % range
    phi_start = -pi/2;
    phi_end   =  pi/2;
    step_rad  =  deg2rad(1);

    best_cost = inf;

    q = [NaN NaN NaN NaN];
    phi_used = NaN;

    % phi and cost
    for phi = phi_start : step_rad : phi_end

        q_temp = ik_pos_with_phi(x,y,z,L1,L2,L3,L4,phi);

        if ~any(isnan(q_temp))

            if isempty(q_prev)
                cost = 0;
            else
                dq = q_temp - q_prev;
                dq = mod(dq + pi, 2*pi) - pi;
                cost = norm(dq)^2;
            end

            if cost < best_cost
                best_cost = cost;
                q = q_temp;
                phi_used = phi;
            end
        end
    end

end

%% Jacobian
    function Jv = jacobian_from_fk(T_all)
        n = numel(T_all);
        Jv = zeros(3,n);
    
        p_e = T_all{end}(1:3,4);
    
        % frame 0
        z_prev = [0;0;1];
        p_prev = [0;0;0];
    
        for i = 1:n
            Jv(:,i) = cross(z_prev, p_e - p_prev);
    
            % update z_prev, p_prev for next joint (use frame i)
            R_i = T_all{i}(1:3,1:3);
            p_i = T_all{i}(1:3,4);
            z_prev = R_i(:,3);
            p_prev = p_i;
        end
    end

%% Joint IK Linear
function moveToJointTarget(q_goal)

    current_q = readJointFromMotor();
    which readJointFromMotor
    assert(numel(current_q)==4, "readJointFromMotor() must return 4 elements, got %d", numel(current_q));

    q_start = current_q(:);
    q_goal  = q_goal(:);
    size(q_start)
    size(q_goal)

    tf = 1.2;
    dt = 0.03;
    tvec = 0:dt:tf;

    for k = 1:numel(tvec)

        t = tvec(k);
        tau = t/tf;

        % cubic time scaling
        s = 3*tau^2 - 2*tau^3;

        q = q_start + s*(q_goal - q_start);

        current_q = q;
        
        % Joint limit
        q_min = [-2.8; -2.0; -2.0; -2.0];
        q_max = [ 2.8;  2.0;  2.0;  2.0];
        current_q = max(min(current_q, q_max), q_min);
        
        sendJointToMotor(current_q);

        % forward kinematics
        [T_all, P_all] = fk_all(current_q, L1, L2, L3, L4);

        % ===== Drawing =====
        cla(ax); hold(ax,'on')

        for i = 1:size(P_all,2)-1
            plot3(ax,...
                [P_all(1,i) P_all(1,i+1)],...
                [P_all(2,i) P_all(2,i+1)],...
                [P_all(3,i) P_all(3,i+1)],...
                'LineWidth',5,'Color',[0.5 0.8 1]);
        end

        for i = 1:size(P_all,2)
            draw_bubble_ui(ax,P_all(:,i),0.015,[0 0 0.8]);
        end

        for i = 1:numel(T_all)
            draw_frame_ui(ax,T_all{i},0.05);
        end

        draw_gripper(ax, P_all(:,end), 0.02, gripper_closed);

        drawnow
    end

end

%% =============== Drawing Helpers ===============
    function draw_frame_ui(ax,T,scale)
        origin = T(1:3,4);
        R = T(1:3,1:3);

        plot3(ax,[origin(1) origin(1)+scale*R(1,1)],...
                 [origin(2) origin(2)+scale*R(2,1)],...
                 [origin(3) origin(3)+scale*R(3,1)],'r','LineWidth',2);

        plot3(ax,[origin(1) origin(1)+scale*R(1,2)],...
                 [origin(2) origin(2)+scale*R(2,2)],...
                 [origin(3) origin(3)+scale*R(3,2)],'g','LineWidth',2);

        plot3(ax,[origin(1) origin(1)+scale*R(1,3)],...
                 [origin(2) origin(2)+scale*R(2,3)],...
                 [origin(3) origin(3)+scale*R(3,3)],'b','LineWidth',2);
    end

    function draw_bubble_ui(ax,center,radius,color)
        [X,Y,Z] = sphere(15);
        surf(ax,...
            radius*X+center(1),...
            radius*Y+center(2),...
            radius*Z+center(3),...
            'FaceColor',color,...
            'EdgeColor','none');
    end

    function draw_gripper(ax, center, size, closed)
    
        offset = 0.015;
    
        if closed
            d = 0.005;
        else
            d = 0.02;
        end
    
        plot3(ax, ...
            [center(1)-d center(1)-d], ...
            [center(2) center(2)], ...
            [center(3)-offset center(3)+offset], ...
            'k','LineWidth',3);
    
        plot3(ax, ...
            [center(1)+d center(1)+d], ...
            [center(2) center(2)], ...
            [center(3)-offset center(3)+offset], ...
            'k','LineWidth',3);
    end

%% Robot Connection
function tick = rad2tick(rad)
    deg  = rad * 180/pi;
    tick = round(deg * 11.375 + 2048);
    tick = max(min(tick, 4095), 0);
end

function rad = tick2rad(tick)
    tick = double(tick);
    deg  = (tick - 2048) / 11.375;
    rad  = deg * pi/180;
end

function sendJointToMotor(q)

    tick1 = rad2tick(q(1));
    tick2 = rad2tick(q(2));
    tick3 = rad2tick(q(3));
    tick4 = rad2tick(q(4));

    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, typecast(int32(tick1),'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, typecast(int32(tick2),'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, typecast(int32(tick3),'uint32'));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, typecast(int32(tick4),'uint32'));

end
    
function q = readJointFromMotor()

    t1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    t2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    t3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PRESENT_POSITION);
    t4 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PRESENT_POSITION);

    t1 = dxlBytesToInt32(t1);
    t2 = dxlBytesToInt32(t2);
    t3 = dxlBytesToInt32(t3);
    t4 = dxlBytesToInt32(t4);

    q = [tick2rad(t1);
         tick2rad(t2);
         tick2rad(t3);
         tick2rad(t4)] + angle_offset;
end

function value = dxlBytesToInt32(data)

    % 如果已经是标量，直接返回
    if numel(data) == 1
        value = double(data);
        return
    end

    % 如果是4个byte
    if numel(data) == 4
        data = uint8(data(:).');          % 保证1x4
        value = double(typecast(data, 'int32'));
        return
    end

    error("Unexpected Dynamixel return size: %d", numel(data));
end

function u = bytes2u32(x)
    if isnumeric(x) && numel(x) == 4
        x = uint8(x(:).');            % 1x4
        u = typecast(x, 'uint32');    % 4 bytes -> uint32
        u = double(u);
    else
        u = double(x);
    end
end

function onClose(~,~)

    ids = [DXL_ID1 DXL_ID2 DXL_ID3 DXL_ID4 DXL_GRIP];
    for id = ids
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, 0);
    end
    closePort(port_num);
    delete(gcf);
end

function setGripper(closed)
    if closed
        tick = 2300; % 你自己标定"闭合"
    else
        tick = 1800; % 你自己标定"张开"
    end
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_GRIP, ADDR_PRO_GOAL_POSITION, typecast(int32(tick),'uint32'));
end

function estop(~,~)
    ids = [DXL_ID1 DXL_ID2 DXL_ID3 DXL_ID4 DXL_GRIP];
    is_estop = true;
    for id = ids
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, 0);
    end
end

function drawInitialRobot()

    % 读取真实电机角度
    q_init = readJointFromMotor();
    q_init = q_init(:);

    % 正运动学
    [T_all, P_all] = fk_all(q_init, L1, L2, L3, L4);

    cla(ax); hold(ax,'on')

    % 画连杆
    for i = 1:size(P_all,2)-1
        plot3(ax,...
            [P_all(1,i) P_all(1,i+1)],...
            [P_all(2,i) P_all(2,i+1)],...
            [P_all(3,i) P_all(3,i+1)],...
            'LineWidth',5,'Color',[0.5 0.8 1]);
    end

    % 画关节球
    for i = 1:size(P_all,2)
        draw_bubble_ui(ax, P_all(:,i), 0.015, [0 0 0.8]);
    end

    % 画坐标系
    for i = 1:numel(T_all)
        draw_frame_ui(ax, T_all{i}, 0.05);
    end

    % 画夹爪
    draw_gripper(ax, P_all(:,end), 0.02, gripper_closed);

    drawnow
end


end