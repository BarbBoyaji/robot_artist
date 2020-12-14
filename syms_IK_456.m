% syms IK 456
syms t4 t5 t6 real
syms pi
rots = simplify(rotz(t4)*rotx(-pi/2)*rotz(t5)*rotx(-pi/2)*rotz(t6))

%t4 = atan2(rots(2,3), rots(1,3)) 

%t5 = acos(- rots(3,3))

%t6 = atan2(rots(3,2) , -rots(3,1))


% rots = simplify(rotz(t4)*roty(t5)*rotx(t6))
