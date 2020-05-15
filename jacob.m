function [jv] = jacob()
    a = 3.14/180 * [0 90 0 90 -90 90];    % link twist vector
    l = [0 0.150 0.590 0.130 0 0];      % link length vector
    d = [0.450 0 0 0.64707 0 0];        % link offset vector
    prompt = 'Input joint angles: '
    th = input(prompt)
    th = 3.14/180 * th;
    
    m = 1;
    for i = 1:6 
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) ...
            -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) ...
            cos(a(i))*d(i);
            0 0 0 1];

        M_tot(1:4,m:m+3) = M;
        m = m+4;
    end
    
    p7 = M(1:3,4);
    
    m = 1;
    Mi0 = eye(4);
    jvi = [];
    for i = 1:6 
        Mi0 = Mi0*M_tot(:,m:m+3);
        ki = Mi0(1:3,3);
        pi = Mi0(1:3,4);
        jvi_0 = [cross(ki,p7-pi);ki];
        jvi = [jvi jvi_0];
        m = m+4;
    end
    
    disp('Jacobian matrix')
    jv = jvi;
end