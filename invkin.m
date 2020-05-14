function sol_tot = invkin()
    prompt = "Input TCP (in meters) in vector form [px py pz]: "
    p40_0 = input(prompt)

    prompt = "Input EulerZYX [fz fy fx]: "
    angles = pi/180 * input(prompt)
%     clc
%     clear all
%     p40_0 = [0.1 0.1 0.05];       %testing values
%     angles = pi/180 * [39 52 78]; %testing values
    
    fi = angles(1);
    theta = angles(2);
    psi_ = angles(3);

    R_ZYX = [cos(fi)*cos(theta)...
        (cos(fi)*sin(theta)*sin(psi_))-(sin(fi)*cos(psi_)) ...
        (cos(fi)*sin(theta)*cos(psi_))+(sin(fi)*sin(psi_));
        sin(fi)*cos(theta) ...
        (sin(fi)*sin(theta)*sin(psi_))+(cos(fi)*cos(psi_)) ...
        (sin(fi)*sin(theta)*cos(psi_))-(cos(fi)*sin(psi_));
        -sin(theta) cos(theta)*sin(psi_) cos(theta)*cos(psi_)];
    R60 = R_ZYX;

    p1 = p40_0(1);
    p2 = p40_0(2);
    p3 = p40_0(3);
    %th1
    th1 = [atan(p2/p1) atan(p2/p1) + pi];
    %th3
    A = -1 - 6.519*((p1./cos(th1) - 0.15).^2 + (p3 - 0.45).^2 - 0.7837);
    B = 9.954;
    C = 1 - 6.519*((p1./cos(th1) - 0.15).^2 + (p3 - 0.45).^2 - 0.7837);

    P = [A(1) B C(1)];
    th3_1 = roots(P);

    P = [A(2) B C(2)];
    th3_2 = roots(P);

    th3 = [th3_1' th3_2'];

    %th2
    syms x y
    a11 = 0.59 + 0.13*cos(th3) + 0.64707*sin(th3);
    a12 = 0.13*sin(th3) - 0.64707*cos(th3);
    a21 = 0.64707*cos(th3) - 0.13*sin(th3);
    a22 = 0.59 + 0.13*cos(th3) + 0.64707*sin(th3);

    b1 = p3 - 0.45;
    b2 = p1./cos(th1) - 0.15;

    s2_0 = [];
    c2_0 = [];
    for i = 1:4
        for j = 1:2
            eq1 = a11(i)*x + a12(i)*y == b1;
            eq2 = a21(i)*x + a22(i)*y == b2(j);

            sol = solve([eq1 eq2], [x y]);
            s2 = sol.x;
            s2_0 = [s2_0 s2];
            c2 = sol.y;
            c2_0 = [c2_0 c2];
        end
    end

    s2 = double(s2_0);
    c2 = double(c2_0);
    t2 = s2./c2;
    th2 = atan(t2);

    th4 = 0;
    th5 = 0;
    th6 = 0;

    k = 1;
    for i = th3
        for j = th1
            R30 = calcr30(i, j, th2(k));
            R63 = R30'*R60;

            th4_1 = atan(R63(3,3)/R63(1,3));
            th4_2 = th4_1 + pi;
            th4 = [th4 th4_1  th4_2];

            cos_5 = R63(2,3);
            sin_5 = sqrt(1-cos_5^2);
            th5_1 = atan(sin_5/cos_5);
            th5_2 = th5_1 + pi;
            th5 = [th5 th5_1 th5_2];

            th6_1 = atan(-R63(2,2)/R63(2,1));
            th6_2 = th6_1 + pi;
            th6 = [th6 th6_1 th6_2];
            k = k+1;
        end
    end

    th1 = 180/pi * th1;
    th2 = 180/pi * th2;
    th3 = 180/pi * th3;
    th4 = 180/pi * th4(1,2:17);
    th5 = 180/pi * th5(1,2:17);
    th6 = 180/pi * th6(1,2:17);

    k = 1;
    int = 0;
    r = 1;
    sol_tot = [sym('i') sym('th1') sym('th2') sym('th3') sym('th4') ...
        sym('th5') sym('th6')];
    
    disp('Joint angles')
    for i =  th3
        if k<8
            for j = th1
                if r == 1
                    k = 0;
                    for m = k+1:k+2
                        for l = k+1:k+2
                            for p = k+1:k+2
                                sol = [sym(num2str(i)) ...
                                    sym(num2str(th2(k+1))) ...
                                    sym(num2str(j)) ...
                                    sym(num2str(th4(m))) ...
                                    sym(num2str(th5(l))) ...
                                    sym(num2str(th6(p)))];
                                int = int + 1;
                                sol_tot = [sol_tot; sym(num2str(int)) sol];
                            end
                        end
                    end
                else
                    for m = k+1:k+2
                        for l = k+1:k+2
                            for p = k+1:k+2
                                sol = [sym(num2str(i)) ...
                                    sym(num2str(th2(k+1))) ...
                                    sym(num2str(j)) ...
                                    sym(num2str(th4(m))) ...
                                    sym(num2str(th5(l))) ...
                                    sym(num2str(th6(p)))];
                                int = int + 1;
                                sol_tot = [sol_tot; sym(num2str(int)) sol];
                            end
                        end
                    end
                end
                r = r+1;
                k = k+1;
            end
        end
    end

    int = int; % must be 64
    disp('Joint angles')
end