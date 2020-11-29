% generate FK symbolic
syms t1 t2 t3 t4 t5 real 
syms L1 L2 L3 L4 L5 real
syms pi real


% DH parameters
% alpha a d theta
DH = [0 0 0 t1;
      0 0 L1 0;
     -pi/2 0 L2 t2;
      0 L3 0 t3;
      pi/2 0 L4 t4;
      0 0 t5 0; 
      -pi/2 0 L5 0];
  
alpha = DH(:,1); 
a = DH(:,2); 
d = DH(:,3); 
theta = DH(:,4);

% initial
To = sym(eye(4)); % base frame itself

for j = 1:7
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = simplify(To*Ti);
    T{j} = To;
    
end

chr = latex(T{7}(1:3,4));
