function M60 = forkin(x)

    a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];       % link twist vector
    l = [0 160 780 150 0 0];                % link length vector
    d = [0 0 0 655 0 0];                 % link offset vector

    th = deg2rad(x);

    index = 1:size(th,2);
    Mt = eye(4);
    for i = index
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1];

        Mt = Mt*M;
    end

    M60 = Mt;
    
    p = M60(1:3,4); % TCP Position in meters

    % Euler ZYX angles are used to describe tip's orientation
    % all angles are displayed in radians
    % disp('Euler ZYX angles')
    %-------------Apo robotics toolbox-----------------%
    f = rotm2eul(M60(1:3,1:3));
end