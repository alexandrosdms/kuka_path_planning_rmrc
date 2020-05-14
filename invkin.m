%function [th1, th2, th3, th4] = invkin()
% prompt = "Input TCP's position in vector form: "
% p40 = input(prompt)
% 
% prompt = "Input Euler ZYX angles for TCP position in vector form: "
% angles = pi/180 * input(prompt)
clc
clear all
p40 = [0.1 0.1 0.05];
angles = pi/180 * [39 52 78];

fi = angles(1);
theta = angles(2);
psi_ = angles(3);

R_ZYX = [cos(fi)*cos(theta) (cos(fi)*sin(theta)*sin(psi_))-(sin(fi)*cos(psi_)) (cos(fi)*sin(theta)*cos(psi_))+(sin(fi)*sin(psi_));
    sin(fi)*cos(theta) (sin(fi)*sin(theta)*sin(psi_))+(cos(fi)*cos(psi_)) (sin(fi)*sin(theta)*cos(psi_))-(cos(fi)*sin(psi_)) ;
    -sin(theta) cos(theta)*sin(psi_) cos(theta)*cos(psi_)];
R60 = R_ZYX;

p1 = p40(1);
p2 = p40(2);
p3 = p40(3);
%th1
th1 = [atan(p2/p1) atan(p2/p1) + pi]
%th3
A = -1 - 6.519*((p1./cos(th1) - 0.15).^2 + (p3 - 0.45).^2 - 0.7837);
B = 9.954;
C = 1 - 6.519*((p1./cos(th1) - 0.15).^2 + (p3 - 0.45).^2 - 0.7837);

P = [A(1) B C(1)];
th3_1 = roots(P);

P = [A(2) B C(2)];
th3_2 = roots(P);

th3 = [th3_1' th3_2']

%th2
syms x y
a11 = 0.59 + 0.13*cos(th3) + 0.64707*sin(th3);
a12 = 0.13*sin(th3) - 0.64707*cos(th3);
a21 = 0.64707*cos(th3) - 0.13*sin(th3);
a22 = 0.59 + 0.13*cos(th3) + 0.64707*sin(th3);

b1 = p3 - 0.45;
b2 = p1./cos(th1) - 0.15;

eq11 = a11(1)*x + a12(1)*y == b1;
eq21 = a21(1)*x + a22(1)*y == b2(1);

eq12 = a11(1)*x + a12(1)*y == b1;
eq22 = a21(1)*x + a22(1)*y == b2(2);

eq13 = a11(2)*x + a12(2)*y == b1;
eq23 = a21(2)*x + a22(2)*y == b2(1);

eq14 = a11(2)*x + a12(2)*y == b1;
eq24 = a21(2)*x + a22(2)*y == b2(2);

eq15 = a11(3)*x + a12(3)*y == b1;
eq25 = a21(3)*x + a22(3)*y == b2(1);

eq16 = a11(3)*x + a12(3)*y == b1;
eq26 = a21(3)*x + a22(3)*y == b2(2);

eq17 = a11(4)*x + a12(4)*y == b1;
eq27 = a21(4)*x + a22(4)*y == b2(1);

eq18 = a11(4)*x + a12(4)*y == b1;
eq28 = a21(4)*x + a22(4)*y == b2(2);

sol1 = solve([eq11, eq21], [x, y]);
sol2 = solve([eq12, eq22], [x, y]);
sol3 = solve([eq13, eq23], [x, y]);
sol4 = solve([eq14, eq24], [x, y]);
sol5 = solve([eq15, eq25], [x, y]);
sol6 = solve([eq16, eq26], [x, y]);
sol7 = solve([eq17, eq27], [x, y]);
sol8 = solve([eq18, eq28], [x, y]);

s2 = [sol1.x sol2.x sol3.x sol4.x sol5.x sol6.x sol7.x sol8.x];
c2 = [sol1.y sol2.y sol3.y sol4.y sol5.y sol6.y sol7.y sol8.y];
s2 = double(s2);
c2 = double(c2);

t2 = s2./c2;
th2 = atan(t2)

th4 = 0;
th5 = 0;
th6 = 0;

k = 1
for i = th3
    for j = th1
        R30 = calcr30(i, j, th2(k));
        R63 = R30'*R60;
        
        th4_1 = atan(R63(3,3)/R63(1,3));
        th4_2 = th4_1 + pi;
        th4 = [th4 th4_1  th4_2]

        cos_5 = R63(2,3);
        sin_5 = sqrt(1-cos_5^2);
        th5_1 = atan(sin_5/cos_5);
        th5_2 = th5_1 + pi;
        th5 = [th5 th5_1 th5_2];

        th6_1 = atan(-R63(2,2)/R63(2,1));
        th6_2 = th6_1 + pi;
        th6 = [th6 th6_1 th6_2];
        k = k+1
    end
end

th4 = 180/pi * th4(1,2:17)
th5 = 180/pi * th5(1,2:17)
th6 = 180/pi * th6(1,2:17)

R30 = calcr30(th3(1), th1(1), th2(1));
R63 = R30'*R60;

th4_1 = atan(R63(3,3)/R63(1,3));
th4_2 = th4_1 + pi;

cos_5 = R63(2,3);
sin_5 = sqrt(1-cos_5^2);
th5_1 = atan(sin_5/cos_5);
th5_2 = th5_1 + pi;

th6_1 = atan(-R63(2,2)/R63(2,1));
th6_2 = th6_1 + pi;

R30 = calcr30(th3(1), th1(2), th2(2));
R63 = R30'*R60;

th4_3 = atan(R63(3,3)/R63(1,3));
th4_4 = th4_3 + pi;

cos_5 = R63(2,3);
sin_5 = sqrt(1-cos_5^2);
th5_3 = atan(sin_5/cos_5);
th5_4 = th5_3 + pi;

th6_3 = atan(-R63(2,2)/R63(2,1));
th6_4 = th6_3 + pi;

R30 = calcr30(th3(2), th1(1), th2(3));
R63 = R30'*R60;

th4_5 = atan(R63(3,3)/R63(1,3));
th4_6 = th4_5 + pi;

cos_5 = R63(2,3);
sin_5 = sqrt(1-cos_5^2);
th5_5 = atan(sin_5/cos_5);
th5_6 = th5_5 + pi;

th6_5 = atan(-R63(2,2)/R63(2,1));
th6_6 = th6_5 + pi;

R30 = calcr30(th3(2), th1(2), th2(4));
R63 = R30'*R60;

th4_7 = atan(R63(3,3)/R63(1,3));
th4_8 = th4_7 + pi;

cos_5 = R63(2,3);
sin_5 = sqrt(1-cos_5^2);
th5_7 = atan(sin_5/cos_5);
th5_8 = th5_7 + pi;

th6_7 = atan(-R63(2,2)/R63(2,1));
th6_8 = th6_7 + pi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


th4 = 180/pi * [th4_1 th4_2 th4_3 th4_4 th4_5 th4_6 th4_7 th4_8]
th5 = 180/pi * [th5_1 th5_2 th5_3 th5_4 th5_5 th5_6 th5_7 th5_8]
th6 = 180/pi * [th6_1 th6_2 th6_3 th6_4 th6_5 th6_6 th6_7 th6_8]
%end