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
% 
% for j = -2*pi:pi/10:2*pi
%     joint = zeros(5,1);
%     joint(1) = j;
%     joint(2) = 0;
%     joint(3) = 0;
%     joint(4) = 0;
%     joint(5) = 0;
%     joint(6) = 0;
%     
%     T_ = FK_6dof(c,joint(:,1));
% 
%     IK_joint = IK_6dof(T_{7}, c);
% 
%     if (IK_joint - joint) > 1e-3
%         disp("ERROR!")
%     end
%     
% end


pt = [eye(3) , [0.15;0.01;0.4]; 0 0 0 1]
IK_joint = IK_6dof(pt, c);
pp = FK_6dof(c,IK_joint);
pp{end}(1:3,4)
