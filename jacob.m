function [jv] = jacob(x)
    if size(x,1) ~= 1
        x = x';
    end
    
    a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];       % link twist vector
    l = [0 160 780 150 0 0];                % link length vector
    d = [0 0 0 655 0 0];
    th = deg2rad(x);
    
    Mt = eye(4);
    index = 1:size(x,2);
    for i = 1:6
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1];

        Mt = Mt*M;
    end

    M60 = Mt;
    %----------------Apo robotics toolbox-------------------------%
    M76 = eul2trm([-pi -pi/2 0]);
    %-------------------------------------------------------------%
    M76(1:3,4) = [125 0 -(250+153)]';
    
    M70 = M60*M76;
    P7 = M70(1:3,4);
    
    jvi = [];
    Mi = eye(4);
    for i = 1:6
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1];
        
        Mi = Mi*M;
        ki = Mi(1:3,3);
        Pi = Mi(1:3,4);
        jvi_0 = [cross(ki,P7-Pi);ki];
        jvi = [jvi jvi_0];
    end
    jv = jvi;
end