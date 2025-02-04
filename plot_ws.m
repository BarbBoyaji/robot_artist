% MAE 263A Project
% Simulation

clc;
close all;
% clf;
clear;

% Parameter
L1 = 0.3; % m
L2 = 0.05; % m
L3 = 0.1; % m
L4 = 0.1; % m


c = [L1, L2, L3, L4, 0];

% Plot
% figure (1)
% i=1;

collection = zeros(3,  2000000);
counter = 1;

for joint1 = -pi:0.1:pi
    for joint2 = -3*pi/2:0.1:pi/2
        for joint3 = -pi:0.1:pi
                
                joint = [joint1, joint2, joint3, 0 , 0, 0];
                
                T = FK_6dof(c,joint);

                pj = T{end}(1:3,4);
                
                collection(:,counter) = pj;
                counter = counter+1;
                
        end
    end
end

collection(:, counter:end) = [];

plot3(collection(1,:),collection(2,:),collection(3,:),'o')
axis equal
rotate3d on
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

% % Manipulator
% for j = 1:5
%     pj = T{j}(1:3,4);
%     pj1 = T{j+1}(1:3,4);
% %     plot3([pj(1) pj1(1)],[pj(2) pj1(2)],[pj(3) pj1(3)],'k','linewidth',4);
% end



% % Frames
% for j = 1:5
%     pj = T{j}(1:3,4);
%     Rj = T{j}(1:3,1:3);
%     scale = 0.025;
%     xj = pj + Rj(:,1)*scale;
%     yj = pj + Rj(:,2)*scale;
%     zj = pj + Rj(:,3)*scale;
%     plot3([pj(1) xj(1)],[pj(2) xj(2)],[pj(3) xj(3)],'r','linewidth',2); % x-axes
%     plot3([pj(1) yj(1)],[pj(2) yj(2)],[pj(3) yj(3)],'g','linewidth',2); % y-axes
%     plot3([pj(1) zj(1)],[pj(2) zj(2)],[pj(3) zj(3)],'b','linewidth',2); % z-axes
% end



% % Base
% plot3([0 0],[0 0],[-0.2 0],'k','linewidth',8);
% 
% % Ground
% X = [1 -1;1 -1]*0.2;
% Y = [1 1;-1 -1]*0.2;
% Z = [1 1;1 1]*-0.2;
% surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none');
% 
% % Setting
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
% pbaspect([1 1 1]);
% grid on;
% 
% view(40,30);
% 
% hold off