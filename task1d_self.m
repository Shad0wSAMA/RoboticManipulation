clear; clc; close all;

L1 = 0.077;
L2 = 0.130;
L3 = 0.124;
L4 = 0.126;

%% figure
figure('Color','w');
axis equal
grid on
view(135,25)
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.6 0.6])
ylim([-0.6 0.6])
zlim([0 0.6])
hold on
camlight headlight
lighting gouraud

%% square def
side = 0.10;
h = side/2;
z = 0.00;
cx = -0.10;
cy = -0.10;

p1 = [cx-h, cy-h, z];
p2 = [cx+h, cy-h, z];
p3 = [cx+h, cy+h, z];
p4 = [cx-h, cy+h, z];

n = 30; %every side 30 points
P = [];

edges = {p1,p2; p2,p3; p3,p4; p4,p1};

for i = 1:4
    A = edges{i,1};
    B = edges{i,2};
    
    t = linspace(0,1,n);
    for k = 1:n
        point = (1-t(k))*A + t(k)*B;
        P = [P; point];
    end
end

P = P';

%% Draw Square
figure('Color','w');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135,25);

line(P(1,:), P(2,:), P(3,:), ...
    'LineWidth',3,'Color',[0 0.5 1]);

xlim([-0.1 0.4]);
ylim([-0.2 0.2]);
zlim([0 0.4]);

%% Animate square tracing

figure(1)  
hold on

for k = 1:size(P,2)
    
    x = P(1,k);
    y = P(2,k);
    z = P(3,k);
    
    % IK
    q = ik_pos_vertical(x,y,z,L1,L2,L3,L4);
    
    % FK
    [T_all,P_all] = fk_all(q,L1,L2,L3,L4);
    
    cla
    
    line(P(1,:), P(2,:), P(3,:), ...
        'LineWidth',3,'Color',[0 0.5 1]);
    
    for i = 1:size(P_all,2)-1
        line([P_all(1,i) P_all(1,i+1)], ...
             [P_all(2,i) P_all(2,i+1)], ...
             [P_all(3,i) P_all(3,i+1)], ...
             'LineWidth',5,'Color',[0.5 0.8 1]);
    end

        % Draw joint bubbles
    for i = 1:size(P_all,2)
        if i == 1
            color = [0 0 0];
        elseif i == size(P_all,2)
            color = [1 0 0];
        else
            color = [0 0 0.8];
        end
        
        draw_bubble(P_all(:,i),0.015,color);
    end
    
    % Draw coordinate frames
    for i = 1:length(T_all)
        draw_frame(T_all{i},0.05);
    end
    
    drawnow
end


%% Test
x = 0.10;
y = 0.10;
z = 0.00;

q = ik_pos_vertical(x,y,z,L1,L2,L3,L4);
[T_all,P_all] = fk_all(q,L1,L2,L3,L4);

P_all(:,end)

%% DH FK
function [T_all, P_all] = fk_all(q, L1, L2, L3, L4)

DH = [ ...
    0   pi/2   L1   q(1);
    L2  0      0    q(2);
    L3  0      0    q(3);
    L4  0      0    q(4)];

T = eye(4);

T_all = {};
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

%% frame
function draw_frame(T, scale)

origin = T(1:3,4);
R = T(1:3,1:3);

x_axis = origin + scale*R(:,1);
y_axis = origin + scale*R(:,2);
z_axis = origin + scale*R(:,3);

line([origin(1) x_axis(1)], ...
     [origin(2) x_axis(2)], ...
     [origin(3) x_axis(3)], ...
     'Color','r','LineWidth',2);

line([origin(1) y_axis(1)], ...
     [origin(2) y_axis(2)], ...
     [origin(3) y_axis(3)], ...
     'Color','g','LineWidth',2);

line([origin(1) z_axis(1)], ...
     [origin(2) z_axis(2)], ...
     [origin(3) z_axis(3)], ...
     'Color','b','LineWidth',2);

end

function draw_bubble(center, radius, color)

[X,Y,Z] = sphere(20);

X = radius*X + center(1);
Y = radius*Y + center(2);
Z = radius*Z + center(3);

surf(X,Y,Z, ...
    'FaceColor',color, ...
    'EdgeColor','none', ...
    'FaceLighting','gouraud');

end

%% IK
function q = ik_pos_vertical(x,y,z,L1,L2,L3,L4)

% 1) yaw
q1 = atan2(y,x);

% 2) planar coordinates
r  = sqrt(x^2 + y^2);
zp = z - L1;

% 3) choose vertical gripper (up)
phi = -pi/2;

% 4) wrist center (vertical)
rw = r - L4*cos(phi);
zw = zp - L4*sin(phi);

% 5) 2R solve for q2,q3
d = sqrt(rw^2 + zw^2);

c3 = (d^2 - L2^2 - L3^2) / (2*L2*L3);
c3 = max(-1,min(1,c3));         % clamp to avoid numerical issues
q3 = atan2( -sqrt(1-c3^2), c3 ); % elbow-down branch (try -acos(c3) for other

alpha = atan2(zw,rw);
beta  = atan2(L3*sin(q3), L2 + L3*cos(q3));
q2 = alpha - beta;

% 6) wrist angle to enforce phi
q4 = phi - q2 - q3;

q = [q1 q2 q3 q4];
end
