clear; clc; close all;

%% Robot link lengths (OpenManipulator approx)
L1 = 0.077;   % base height
L2 = 0.130;   % link 2
L3 = 0.124;   % link 3
L4 = 0.126;   % gripper length

%% Joint limits (modify to match real limits)
theta1_range = linspace(-pi, pi, 40);
theta2_range = linspace(-pi/2, pi/2, 40);
theta3_range = linspace(-pi/2, pi/2, 40);

%% Storage
points = [];

%% Sweep joint space
for t1 = theta1_range
    for t2 = theta2_range
        for t3 = theta3_range
            
            % enforce downward gripper
            t4 = -pi/2 - t2 - t3;
            
            % optional: joint 4 limits check
            if t4 < -pi/2 || t4 > pi/2
                continue
            end
            
            % ---- Forward Kinematics ----
            
            % planar reach
            r = L2*cos(t2) + L3*cos(t2+t3) + L4*cos(t2+t3+t4);
            z = L1 ...
              + L2*sin(t2) ...
              + L3*sin(t2+t3) ...
              + L4*sin(t2+t3+t4);
            
            x = r*cos(t1);
            y = r*sin(t1);
            
            points = [points; x y z];
        end
    end
end

%% Plot reachable workspace
figure('Color','w')
scatter3(points(:,1), points(:,2), points(:,3), 5, 'filled')
axis equal
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Reachable Workspace (Gripper Fixed Downward)')
view(135,25)

figure
scatter(points(:,1), points(:,2), 5, 'filled')
axis equal
grid on
xlabel('X')
ylabel('Y')
title('XY Projection (Gripper Down)')
