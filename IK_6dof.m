function joint = IK_6dof(T0e,c)
% T0e - end-effector transform matrix
% c - parameter
% joint - joint position list

L1 = c(1);
L2 = c(2);
L3 = c(3);
L4 = c(4);


   
T06 = T0e;%/T6e;
px = T06(1,4);
py = T06(2,4);
pz = T06(3,4);

r0 = sqrt(px^2 + py^2);
cos_phi = (-py^2 + (px^2 + r0^2))/(2*px*r0);
sin_phi = sqrt(1-cos_phi^2);
if(py < 0)
    phi = atan2(-py, px);
    beta = acos(L2/r0);
    t1 = - (pi/2 - (beta - phi));
else
    phi = atan2(sin_phi, cos_phi);
    beta = acos(L2/r0);
    t1 = phi + beta - pi/2;
end


p_rot = sqrt(r0^2 - L2^2);
r3 = sqrt(p_rot^2 + (L1-pz)^2);
cos_t3 = (r3^2 - (L3^2+L4^2))/(2*L3*L4);
sin_t3 = sqrt(1-cos_t3^2);
t3 = atan2(sin_t3, cos_t3);


r2 = sqrt(p_rot^2 +(pz-L1)^2);
cos_psi = (-L4^2 + (r2^2 + L3^2))/(2*r2*L3);
cos_gamma = (-(pz-L1)^2 + (p_rot^2 + r2^2))/(2*p_rot*r2);

if pz > L1
    t2 = - acos(cos_psi) - acos(cos_gamma);
else
    t2 = - acos(cos_psi) + acos(cos_gamma);
end

% use euler angles to get the orientation of t4, t5, t6
% z(t1) -> x(-90) -> z(t2) -> z(t3) -> x(90)
R01 = [cos(t1) -sin(t1) 0;
       sin(t1) cos(t1) 0;
       0 0 1];
R12 = [1 0 0;
       0 cos(-pi/2) -sin(-pi/2);
       0 sin(-pi/2) cos(-pi/2)]*[cos(t2) -sin(t2) 0;
       sin(t2) cos(t2) 0;
       0 0 1];
R23 = [cos(t3) -sin(t3) 0;
       sin(t3) cos(t3) 0;
       0 0 1];
   
R34 = [1 0 0;
       0 cos(pi/2) -sin(pi/2);
       0 sin(pi/2) cos(pi/2)];
   
eef = (R01*R12*R23*R34)'*T06(1:3,1:3);

t5 = atan2(sqrt(eef(1,3)^2 + eef(2,3)^2),-eef(3,3));

s5 = sin(t5);

t4 = atan2(-eef(2,3)/s5,-eef(1,3)/s5);
t6 = atan2(eef(3,2)/s5,-eef(3,1)/s5);

joint = [t1;t2;t3;t4;t5;t6];

end