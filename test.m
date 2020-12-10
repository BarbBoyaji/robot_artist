% MAE 263A Project
% Simulation

clc;
% clf;
clear;

% Parameter 
L1 = 0.3; % m
L2 = 0.05; % m
L3 = 0.1; % m
L4 = 0.1; % m


c = [L1, L2, L3, L4];

% Plot
figure ()
i=1;

joint = zeros(5,1);
joint(1) = -pi/4;
joint(2) = 0;
joint(3) = 0;
joint(4) = 0;
joint(5) = 0;
joint(6) = 0;
T_ = FK_6dof(c,joint(:,i));

joint = IK_6dof(T_{7}, c);

T = FK_6dof(c,joint(:,i));

hold on

% Manipulator
for j = 1:5
    pj = T{j}(1:3,4);
    pj1 = T{j+1}(1:3,4);
    plot3([pj(1) pj1(1)],[pj(2) pj1(2)],[pj(3) pj1(3)],'k','linewidth',4);
end

% Frames
for j = 1:7
    pj = T{j}(1:3,4);
    Rj = T{j}(1:3,1:3);
    scale = 0.025;
    xj = pj + Rj(:,1)*scale;
    yj = pj + Rj(:,2)*scale;
    zj = pj + Rj(:,3)*scale;
    plot3([pj(1) xj(1)],[pj(2) xj(2)],[pj(3) xj(3)],'r','linewidth',2); % x-axes
    plot3([pj(1) yj(1)],[pj(2) yj(2)],[pj(3) yj(3)],'g','linewidth',2); % y-axes
    plot3([pj(1) zj(1)],[pj(2) zj(2)],[pj(3) zj(3)],'b','linewidth',2); % z-axes
end

% Base
plot3([0 0],[0 0],[-0.2 0],'k','linewidth',8);

% Ground
X = [1 -1;1 -1]*0.2;
Y = [1 1;-1 -1]*0.2;
Z = [1 1;1 1]*-0.2;
surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none');

% Setting
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ranges = 0.4;
axis([-ranges ranges -ranges ranges -ranges ranges]);
%pbaspect([1 1 1]);
axis equal
grid on;

view(40,30);

%hold off


%% plot the sketch

% test code to plot the first graph
z(1)=0;
y(1)=0;

N=100;
traj = linspace(0,50,N);

for k = 1:N
z(k+1)=(y(k)*(1+sind(0.7*z(k))))-1.2*sqrt(abs(z(k)));
y(k+1)=0.21-z(k);
end
x = .15*ones(1,k+1);

%rose 1
plot3(x,.1*y,.1*z+.2, 'r')

%rose 2
plot3(x,.1*y +.1,.1*z+.15, 'r')

%rose 2
plot3(x,.1*y -.1,.1*z+.15, 'r')

%% animation

% Trajectory in Cartesian Space
N = 100;
t = linspace(0,2*pi,N);
x = 0.025*cos(t) + 0.1;
y = 0.025*sin(t);
z = 0.005*sin(5*t) + 0.2;

figure()
plot3(x,y,z,'r');


% Joint Space
for i = 1:N
    p = [x(i);y(i);z(i)];
    R = [-1 0 0;0 1 0;0 0 -1];
    T0e = [R     p;
           0 0 0 1];
    joint(:,i) = IK_6dof(T0e,c);
end


% Plot Setting
movie = 1;
speed = 1;

% Create Movie
if movie == 1
    v = VideoWriter('ani.avi');
    open(v);
end

% Plot
figure

for i = 1:speed:N

T = FK_6dof(c,joint(:,i));

% Plot Trajectory
plot3(x,y,z,'r');
hold on;

% Manipulator
for j = 1:5
    pj = T{j}(1:3,4);
    pj1 = T{j+1}(1:3,4);
    plot3([pj(1) pj1(1)],[pj(2) pj1(2)],[pj(3) pj1(3)],'k','linewidth',4);
end

% Tool
p6 = T{6}(1:3,4);
p7 = T{7}(1:3,4);
plot3([p6(1) p7(1)],[p6(2) p7(2)],[p6(3) p7(3)],'m','linewidth',3);

% Frames
for j = 1:7
    pj = T{j}(1:3,4);
    Rj = T{j}(1:3,1:3);
    scale = 0.025;
    xj = pj + Rj(:,1)*scale;
    yj = pj + Rj(:,2)*scale;
    zj = pj + Rj(:,3)*scale;
    plot3([pj(1) xj(1)],[pj(2) xj(2)],[pj(3) xj(3)],'r','linewidth',2); % x-axes
    plot3([pj(1) yj(1)],[pj(2) yj(2)],[pj(3) yj(3)],'g','linewidth',2); % y-axes
    plot3([pj(1) zj(1)],[pj(2) zj(2)],[pj(3) zj(3)],'b','linewidth',2); % z-axes
end

% Base
plot3([0 0],[0 0],[-0.2 0],'k','linewidth',8);

% Ground
X = [1 -1;1 -1]*0.2;
Y = [1 1;-1 -1]*0.2;
Z = [1 1;1 1]*-0.2;
surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none');

% Setting
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis([-0.2 0.2 -0.2 0.2 -0.2 0.5]);
pbaspect([1 1 1]);
grid on;

view(40,30);
%view(2);

hold off

if movie == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
else
    drawnow
end

end

if movie == 1
    close(v)
end