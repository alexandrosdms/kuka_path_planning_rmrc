function [R30] = calcr03(theta1, theta2, theta3);
    a = pi/180 * [0 90 0 90 -90 90];    % link twist vector
    th = [theta1 theta2 theta3];
    %dh_par = [index' a' l' d' th'];

    R_tot = zeros(3,12);
    m = 1;
    % All matrices will be stored here 
    for k = 1:3  
        R = [cos(th(k)) -sin(th(k)) 0;
            sin(th(k))*cos(a(k)) cos(th(k))*cos(a(k)) -sin(a(k));
            sin(th(k))*sin(a(k)) cos(th(k))*sin(a(k)) cos(a(k))];
        R_tot(:,m:m+2) = R;
        m = m+3;
    end

    R10 = R_tot(:,1:3);      % 1 to 0 transformation
    R21 = R_tot(:,4:6);      % 2 to 1    >>
    R32 = R_tot(:,7:9);     % 3 to 2    >>

    R30 = R32*R21*R10;
end

