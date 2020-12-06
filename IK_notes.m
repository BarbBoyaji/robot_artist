% IK note

% first triangle 
% joint 1,2 &3 
% for T06

% second triangle
% joint 4& 5
% for T57


% results of FK:
% R = [ cos(t2 + t3)*cos(t1)*cos(t4) - sin(t1)*sin(t4), -sin(t2 + t3)*cos(t1), - cos(t4)*sin(t1) - cos(t2 + t3)*cos(t1)*sin(t4)]
%     [ cos(t1)*sin(t4) + cos(t2 + t3)*cos(t4)*sin(t1), -sin(t2 + t3)*sin(t1),   cos(t1)*cos(t4) - cos(t2 + t3)*sin(t1)*sin(t4)]
%     [                          -sin(t2 + t3)*cos(t4),         -cos(t2 + t3),                             sin(t2 + t3)*sin(t4)]


% P =   L4*sin(t2 + t3)*cos(t1) - L5*(cos(t4)*sin(t1) + cos(t2 + t3)*cos(t1)*sin(t4)) - L2*sin(t1) + L3*cos(t1)*cos(t2)
%       L2*cos(t1) + L5*(cos(t1)*cos(t4) - cos(t2 + t3)*sin(t1)*sin(t4)) + L4*sin(t2 + t3)*sin(t1) + L3*cos(t2)*sin(t1)
%       L1 + L4*cos(t2 + t3) - L3*sin(t2) + L5*sin(t2 + t3)*sin(t4)

% given px py
% theta1 = atan2(py, px) - theta_rho


% theta4 = 



