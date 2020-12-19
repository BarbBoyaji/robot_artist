function T = FK_6dof(c,joint)
% c - parameter
% joint - joint position list
% T - T{j} = T0j

L1 = c(1);
L2 = c(2);
L3 = c(3);
L4 = c(4);


t1 = joint(1);
t2 = joint(2);
t3 = joint(3);
t4 = joint(4);
t5 = joint(5);
t6 = joint(6);
 
DH = [0 0 0 t1;
      0 0 L1 0;
      -pi/2 0 L2 t2;
      0 L3 0 t3;
      pi/2 L4 0 t4;
      -pi/2 0 0 t5
      -pi/2 0 0 t6];


alpha = DH(:,1); 
a = DH(:,2); 
d = DH(:,3); 
theta = DH(:,4);

% initial
To = eye(4); % base frame itself

for j = 1:7
    
    Ti = [cos(theta(j))                 -sin(theta(j))                0                 a(j);
          sin(theta(j))*cos(alpha(j))   cos(theta(j))*cos(alpha(j))  -sin(alpha(j))    -sin(alpha(j))*d(j);
          sin(theta(j))*sin(alpha(j))   cos(theta(j))*sin(alpha(j))   cos(alpha(j))    cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    T{j} = To;
    
end

end