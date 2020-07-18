clc

syms q1 real;
syms q2 real;
syms q3 real;
syms q4 real;
syms q5 real;
syms q6 real;
syms M_tot;

a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];       % link twist vector
l = [0 150 590 130 0 0];      % link lengq vector
d = [0 0 0 647.07 0 0];       % link offset vector
q = [q1 q2 q3 q4 q5 q6];
% a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];       % link twist vector
% l = [0 0 a2 a3 0 0];      % link lengq vector
% d = [0 0 d3 d4 0 0];       % link offset vector
% q = [q1 q2 q3 q4 q5 q6];

index = 1:6;

% puma test of transformation matrices
% dh_par = [index' a' l' d' q'];
% dh_par(1:6,3:4) = vpa(dh_par(1:6,3:4));
% dh_par

m = 1;
% All matrices will be stored here 
for k = index  
        M = [cos(q(k)) -sin(q(k)) 0 l(k);
            sin(q(k))*cos(a(k)) cos(q(k))*cos(a(k)) -sin(a(k)) -sin(a(k))*d(k);
            sin(q(k))*sin(a(k)) cos(q(k))*sin(a(k)) cos(a(k)) cos(a(k))*d(k);
            0 0 0 1];
        
        for i = 2:3
            for j = 1:2
                if(j == 1)
                    if abs(double(M(i,j)/sin(q(k))))<0.0001
                        M(i,j) = sym('0');
                    end
                elseif(j == 2)
                    if abs(double(M(i,j)/cos(q(k))))<0.0001
                        M(i,j) = sym('0');
                    end
                end
            end
        end
        
        for i = 1:3
            for j = 3:4
                r = double(M(i,j));
                if abs(r) < 0.0001
                  M(i,j) = sym('0');
                else
                    r = num2str(r);
                    M(i,j) = sym(r);
                end
            end
        end
% M0 = M
M_tot(1:4,m:m+3) = M;
m = m+4;
end

M10 = M_tot(:,1:4)     % 1 to 0 transformation
M21 = M_tot(:,5:8)     % 2 to 1    >>
M32 = M_tot(:,9:12)    % 3 to 2    >>
M43 = M_tot(:,13:16)   % 4 to 3    >>
M54 = M_tot(:,17:20)   % 5 to 4    >>
M65 = M_tot(:,21:24)   % 6 to 5    >>

M60 = M10*M21*M32*M43*M54*M65

syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz
M60_ = [r11 r12 r13 px;r21 r22 r23 py;r31 r32 r33 pz;0 0 0 1]

M01 = simplify(inv(M10))
M61_ = M01*M60_
M61 = M21*M32*M43*M54*M65

M30 = M10*M21*M32;
M03 = inv(M30);
M03 = simplify(M03)
M63_ = M03*M60_
M63 = M43*M54*M65

M64_ = simplify(inv(simplify(M10*M21*M32*M43)))*M60_
M64 = M65

M65_ = simplify(inv(simplify(M10*M21*M32*M43*M54)))*M60_
M65 = M65
% M30 = M10*M21*M32;
% p43 = M43(:,4);
% a = M30*p43;
% f1 = partfrac(a(1)/cos(q1))-0.15;
% f1 = vpa(f1);
% f1 = 0.59*cos(q2)+0.13*cos(q2 + q3)-0.64707*sin(q2 + q3);
% 
% f3 = vpa(a(3));
% f3 = -0.64707*cos(q2+q3)-0.13*sin(q2+q3)-0.59*sin(q2);
% f = f1^2 + f3^2;
% f = simplify(f);
% f = vpa(f);
% f = 0.7788*cos(q3 + 1.3725) + 0.7837;
% f = vpa(f);
% s = (px/cos(q1)-0.15)^2 + pz^2;
% s = subs(s,q1,th11);
% s = vpa(s);
% eqq3 = f == s
% th3 = solve(eqq3,q3);
% th3 = double(th3)
% f3 = subs(f3,q3,th3(2));
% f3 = vpa(f3);
% f3 = simplify(f3);
% 
% eq2 = f3 == pz;
% th2 = solve(eq2);
% th2 = double(th2);
% 
% R63 = simplify(M43(1:3,1:3)*M54(1:3,1:3)*M65(1:3,1:3));
% R30 = M30(1:3,1:3);
% R30_inv1 = R30';
% b = R30_inv1*R60;
% b = subs(b,{q1,q2,q3},{th11,th2(1),th3(2)});
% b = vpa(b);
% 
% b13 = b(1,3);
% b33 = b(3,3);
% eq4 = b33/b13 == R63(3,3)/R63(1,3);
% th4 = solve(eq4);
% th4 = double(th4);
% eq5 = b(2,3) == R63(2,3);
% th5 = solve(eq5);
% th5 = double(th5);
% b21 = b(2,1);
% b22 = b(2,2);
% eq6 = b22/b21 == R63(2,2)/R63(2,1);
% th6 = solve(eq6);
% th6 = double(th6);
% 
% sol = [th1(1) th2(1) th3(1) th4(1) th5(1) th6(1)]

