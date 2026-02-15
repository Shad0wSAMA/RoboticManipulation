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


%% Video

v = VideoWriter('robot_fk_demo.mp4','MPEG-4');
open(v);

%% Simultaneous motion of all joints

for t = linspace(0,2*pi,180)
    
    cla
    
    q1 = pi/4 * sin(t);
    q2 = pi/6 * sin(2*t);
    q3 = pi/6 * cos(1.5*t);
    q4 = pi/8 * sin(3*t);
    
    q = [q1 q2 q3 q4];
    
    [T_all, P_all] = fk_all(q, L1, L2, L3, L4);
    
    % Draw robot links (light blue)
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
    frame = getframe(gcf);
    writeVideo(v,frame);
end


close(v);
disp("Video saved as robot_fk_demo.mp4");

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
