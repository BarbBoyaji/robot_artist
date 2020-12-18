% IK alge
clear all
% generate FK symbolic
syms t1 t2 t3 t4 t5 t6 real 
syms L1 L2 L3 L4 L5 real



% DH parameters
% alpha a d theta
DH = sym([0 0 L1 t1;
      -pi/2 0 L2 t2;
      0 L3 0 t3;
      pi/2 0 L4 t4;
      -pi/2 0 0 t5
      -pi/2 0 0 t6]);
  
alpha = DH(:,1); 
a = DH(:,2); 
d = DH(:,3); 
theta = DH(:,4);

% initial
To = sym(eye(4)); % base frame itself

for j = 1:6
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    T{j} = To;
    
end

syms a1 a2 a3 a4 a5 real real
syms al1 al2 al3 al4 al5 real
syms d1 d2 d3 d4 real
syms s1 s2 s3 s4 s5 

a1 = 0;
al2 = 0;
a3 = 0;

f1 = a3*cos(t3) + d4* sin(al3)*sin(t3) + a2
f2 = a3*c(al2)*sin(t3) - d4*sin(alpha3)

px = T{end}(1,4)
py = T{end}(2,4)
pz = T{end}(3,4)

r = simplify(expand(px^2 + py^2 + pz^2))