function [] = forkin();
    %input: a_i-1, l_i-1, d_i, th_i
    % output: dh_par, M10, M21, M32, M43, M54, M65

    a = pi/180 * [0 90 0 90 -90 90];    % link twist vector
    l = [0 0.150 0.590 0.130 0 0];      % link length vector
    d = [0.450 0 0 0.64707 0 0];        % link offset vector

    prompt = 'Input joint angles in vector form: '

    th = pi/180 * input(prompt);        % joint angle vector
    index = 1:6;

    %dh_par = [index' a' l' d' th'];

    m = 1;
    M_tot = zeros(4,16);    % All matrices will be stored here
    for i = index  
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1];

        M_tot(1:4,m:m+3) = M;
        m = m+4;
    end

    M10 = M_tot(:,1:4)      % 1 to 0 transformation
    M21 = M_tot(:,5:8)      % 2 to 1    >>
    M32 = M_tot(:,9:12)     % 3 to 2    >>
    M43 = M_tot(:,13:16)    % 4 to 3    >>
    M54 = M_tot(:,17:20)    % 5 to 4    >>
    M65 = M_tot(:,21:24)    % 6 to 5    >>

    M60 = M65*M54*M43*M32*M21*M10;  % tip to base transformation matrix

    p = M60(1:3,4)  % Position of arm's tip in meters

    % Euler ZYX angles are used to describe tip's orientation
    % all angles are displayed in radians
    fi = atan(M60(2,1)/M60(1,1))    % Euler Z angle 
    theta = atan(-M60(3,1)/sqrt((M60(1,1)^2 + M60(2,1)^2))) % Euler Y angle
    psi_ = atan(M60(2,3)/M60(3,3))   % Euler X angle
end