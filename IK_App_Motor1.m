function IK_App_Motor1

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
L4 = 0.126;

scaleXY = 0.2 / 7.5;

%% Home Position
x_home = 0.03;
y_home = 0.00;
z_home = 0.15;

q_home = [0;
          2.059;
         -1.313;
         -2.317];

%% current angle

% handle for previous path line
pathHandle = [];

% ===== Read real motor position as initial state =====

sendJointToMotor([0 0 0 0]);

current_q = readJointFromMotor();
current_q = current_q(:);

%% current angle (init can omit q_prev)
[current_q, ~] = ik_search_phi(x_home,y_home,z_home,L1,L2,L3,L4);

% handles
pathHandle = [];     % final executed path (black dashed)
refHandle  = [];     % reference straight line (red dashed)
robotHandles = gobjects(0,1); % graphics objects for robot (so we can delete each frame)

%% Create Main UI Window
fig = uifigure('Name','OpenManipulator IK Control',...
               'Position',[300 200 900 500]);

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
        x = xField.Value;
        y = yField.Value;
        z = zField.Value;
        moveToTarget(x,y,z);
    end

%% Core motion + draw + path (Hybrid: only bad segment uses joint interpolation)
    function moveToTarget(x,y,z)

        r_sing = 0.03;     % singular cylinder radius
        N      = 120;      % number of points to analyse along straight line
        tf_seg = 0.6;      % time per segment
        dt     = 0.03;     % animation step

        % delete last executed path line
        if ~isempty(pathHandle) && isvalid(pathHandle)
            delete(pathHandle);
            pathHandle = [];
        end

        % delete last reference line
        if ~isempty(refHandle) && isvalid(refHandle)
            delete(refHandle);
            refHandle = [];
        end

        % compute start tool position
        [~, P_initial] = fk_all(current_q, L1, L2, L3, L4);
        p0 = P_initial(:, end);
        pf = [x; y; z];

        % quick reachability check for final target (optional)
        [q_goal, phi_goal] = ik_search_phi(x,y,z,L1,L2,L3,L4, current_q);
        if any(isnan(q_goal))
            thetaLabel.Value = "Unreachable target!";
            return;
        end

        thetaLabel.Value = sprintf([ ...
            'Target IK:\n' ...
            'θ1 = %.3f rad (%.1f°)\n' ...
            'θ2 = %.3f rad (%.1f°)\n' ...
            'θ3 = %.3f rad (%.1f°)\n' ...
            'θ4 = %.3f rad (%.1f°)\n' ...
            'φ selected = %.3f rad (%.1f°)'], ...
            q_goal(1),rad2deg(q_goal(1)), ...
            q_goal(2),rad2deg(q_goal(2)), ...
            q_goal(3),rad2deg(q_goal(3)), ...
            q_goal(4),rad2deg(q_goal(4)), ...
            phi_goal,rad2deg(phi_goal));

        % build reference straight line points for segmentation
        s_lin = linspace(0,1,N);
        P = zeros(3,N);
        for i = 1:N
            P(:,i) = p0 + s_lin(i) * (pf - p0);
        end

        % plot reference line (red dashed)
        refHandle = plot3(ax, P(1,:), P(2,:), P(3,:), 'r--', 'LineWidth', 2);

        % compute r along path and mark bad region
        r_all = vecnorm(P(1:2,:), 2, 1);   % 1xN
        bad = (r_all < r_sing);

        % find continuous bad segments [start,end]
        d = diff([0 bad 0]);
        bad_starts = find(d==1);
        bad_ends   = find(d==-1)-1;

        % create segments list [a b type], type=0 safe (cartesian), 1 bad (joint)
        segments = [];
        idx = 1;
        for k = 1:numel(bad_starts)
            bs = bad_starts(k);
            be = bad_ends(k);

            if bs-1 >= idx
                segments = [segments; idx, bs-1, 0];
            end
            segments = [segments; bs, be, 1];
            idx = be+1;
        end
        if idx <= N
            segments = [segments; idx, N, 0];
        end

        % storage for executed path (black dashed)
        executed = [];

        % execute each segment
        for si = 1:size(segments,1)

            a = segments(si,1);
            b = segments(si,2);
            isBad = segments(si,3);

            pA = P(:,a);
            pB = P(:,b);

            if isBad == 0
                % ==============================
                % SAFE segment: Cartesian + IK
                % ==============================
                tvec = 0:dt:tf_seg;
                for kk = 1:numel(tvec)
                    tau = tvec(kk)/tf_seg;
                    s = 3*tau^2 - 2*tau^3;
                    p = pA + s*(pB - pA);

                    [q, ~] = ik_search_phi(p(1),p(2),p(3), L1,L2,L3,L4, current_q);
                    if any(isnan(q))
                        thetaLabel.Value = "Path blocked (IK fail on safe seg).";
                        break;
                    end

                    current_q = q; % continuity

                    [T_all,P_all] = fk_all(q,L1,L2,L3,L4);
                    executed(:,end+1) = P_all(:,end); %#ok<AGROW>

                    draw_robot(T_all, P_all);
                end

            else
                % ==========================================
                % BAD segment: joint-space interpolation ONLY
                % Choose two boundary joint configs from nearby SAFE points
                % ==========================================

                % find a safe point just before a, if none use a
                a_safe = max(1, a-1);
                while a_safe >= 1 && bad(a_safe)
                    a_safe = a_safe - 1;
                end
                if a_safe < 1
                    a_safe = a;
                end

                % find a safe point just after b, if none use b
                b_safe = min(N, b+1);
                while b_safe <= N && bad(b_safe)
                    b_safe = b_safe + 1;
                end
                if b_safe > N
                    b_safe = b;
                end

                pA_safe = P(:,a_safe);
                pB_safe = P(:,b_safe);

                % IK for boundary configs with continuity
                [qA, ~] = ik_search_phi(pA_safe(1),pA_safe(2),pA_safe(3), L1,L2,L3,L4, current_q);
                if any(isnan(qA))
                    thetaLabel.Value = "Blocked near bad segment start.";
                    break;
                end
                current_q = qA;

                [qB, ~] = ik_search_phi(pB_safe(1),pB_safe(2),pB_safe(3), L1,L2,L3,L4, current_q);
                if any(isnan(qB))
                    thetaLabel.Value = "Blocked near bad segment end.";
                    break;
                end

                % joint interpolation through this bad section
                tvec = 0:dt:tf_seg;
                for kk = 1:numel(tvec)
                    tau = tvec(kk)/tf_seg;
                    s = 3*tau^2 - 2*tau^3;
                    q = qA + s*(qB - qA);

                    current_q = q;

                    [T_all,P_all] = fk_all(q,L1,L2,L3,L4);
                    executed(:,end+1) = P_all(:,end); %#ok<AGROW>

                    draw_robot(T_all, P_all);
                end

                % ensure end state
                current_q = qB;
            end
        end

        % draw executed path (black dashed)
        if ~isempty(executed)
            pathHandle = plot3(ax, executed(1,:), executed(2,:), executed(3,:), 'k--', 'LineWidth', 1.5);
        end
    end

%% Callback: Return Home
    function goHome(~,~)
        moveToTarget(x_home,y_home,z_home);
    end

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

        s3 = -sqrt(1-c3^2); % elbow-down branch
        q3 = atan2(s3,c3);

        alpha = atan2(zw,rw);
        beta  = atan2(L3*s3, L2 + L3*c3);
        q2 = alpha - beta;

        q4 = phi - q2 - q3;

        q = [q1 q2 q3 q4];
    end

%% Search available phi (continuity by q_prev, no reject here!)
    function [q, phi_used] = ik_search_phi(x,y,z,L1,L2,L3,L4, q_prev)

        if nargin < 8
            q_prev = [];
        end

        phi_start = -pi/2;
        phi_end   =  pi/2;
        step_rad  =  deg2rad(1);

        best_cost = inf;
        q = [NaN NaN NaN NaN];
        phi_used = NaN;

        for phi = phi_start : step_rad : phi_end

            q_temp = ik_pos_with_phi(x,y,z,L1,L2,L3,L4,phi);

            if ~any(isnan(q_temp))
                if isempty(q_prev)
                    cost = 0;
                else
                    dq = q_temp - q_prev;
                    dq = wrapToPi_local(dq);
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

%% =============== Drawing Helpers ===============
function draw_robot(T_all, P_all)

    % delete previous robot graphics
    if ~isempty(robotHandles)
        delete(robotHandles(isvalid(robotHandles)));
    end
    robotHandles = gobjects(0,1);

    % links
    for i = 1:size(P_all,2)-1
        h = plot3(ax,...
            [P_all(1,i) P_all(1,i+1)],...
            [P_all(2,i) P_all(2,i+1)],...
            [P_all(3,i) P_all(3,i+1)],...
            'LineWidth',5,'Color',[0.5 0.8 1]);
        robotHandles(end+1,1) = h;
    end

    % joints
    for i = 1:size(P_all,2)
        h = draw_bubble_ui(ax,P_all(:,i),0.015,[0 0 0.8]);
        robotHandles(end+1,1) = h;
    end

    % frames
    for i = 1:numel(T_all)
        hh = draw_frame_ui(ax,T_all{i},0.05);
        robotHandles = [robotHandles; hh(:)];
    end

    drawnow
end

    function hh = draw_frame_ui(ax,T,scale)
        origin = T(1:3,4);
        R = T(1:3,1:3);

        h1 = plot3(ax,[origin(1) origin(1)+scale*R(1,1)],...
                      [origin(2) origin(2)+scale*R(2,1)],...
                      [origin(3) origin(3)+scale*R(3,1)],'r','LineWidth',2);

        h2 = plot3(ax,[origin(1) origin(1)+scale*R(1,2)],...
                      [origin(2) origin(2)+scale*R(2,2)],...
                      [origin(3) origin(3)+scale*R(3,2)],'g','LineWidth',2);

        h3 = plot3(ax,[origin(1) origin(1)+scale*R(1,3)],...
                      [origin(2) origin(2)+scale*R(2,3)],...
                      [origin(3) origin(3)+scale*R(3,3)],'b','LineWidth',2);
        hh = [h1; h2; h3];
    end

    function h = draw_bubble_ui(ax,center,radius,color)
        [X,Y,Z] = sphere(15);
        h = surf(ax,...
            radius*X+center(1),...
            radius*Y+center(2),...
            radius*Z+center(3),...
            'FaceColor',color,...
            'EdgeColor','none');
    end

%% Local wrapToPi (no toolbox needed)
    function a = wrapToPi_local(a)
        a = mod(a + pi, 2*pi) - pi;
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
