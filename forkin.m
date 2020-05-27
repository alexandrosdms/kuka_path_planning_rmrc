function [Mt] = forkin(x, tool)
    %input: a_i-1, l_i-1, d_i, th_i
    % output: pinakes metasximatismou Mi_i-1
    a6 = 0;
    l6 = 0;
    tlength = 0.205;
    d7 = tlength + 0.095; % 0.095 mikos provoskidas

    a = pi/180 * [0 90 0 90 -90 90 a6];    % link twist vector
    l = [0 0.150 0.590 0.130 0 0 l6];      % link length vector
    d = [0.450 0 0 0.64707 0 0 d7];        % link offset vector
    
    if nargin == 0
        prompt = 'Input joint angles in vector form: '
        x = deg2rad(input(prompt));   % joint angle vector
        th = x;
    elseif nargin == 2
        if tool == false
            th = deg2rad(x);
        elseif tool == true
            th = deg2rad([x 0]); % den yparxei sxetiki peristrofi metaxi {6} k {7}
        end
    else
        disp('ERROR: 2 OR NONE INPUT ARGUMENTS MUST BE GIVEN')
        return;
    end
    
    index = 1:size(th,2);

    Mt = eye(4);
    for i = index  
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) ...
            -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) ...
            cos(a(i))*d(i);
            0 0 0 1];
        
        Mt = Mt*M;
    end
    
    %disp('TCP in meters')
    p = Mt(1:3,4);  % Position of arm's tip in meters

    % Euler ZYX angles are used to describe tip's orientation
    % all angles are displayed in radians
    %disp('Euler ZYX angles')
    fz = atan(Mt(2,1)/Mt(1,1));    % Euler Z angle 
    fy = atan(-Mt(3,1)/sqrt((Mt(1,1)^2 + Mt(2,1)^2))); % Euler Y angle
    fx = atan(Mt(2,3)/Mt(3,3));   % Euler X angle
end