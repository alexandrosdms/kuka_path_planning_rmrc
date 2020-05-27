function R_ZYX = R_ZYX(z,y,x)
    switch nargin
        case 3
            z = deg2rad(z);
            y = deg2rad(y);
            x = deg2rad(x);
            
        case 1
            zeta = z;
            z = deg2rad(zeta(1));
            y = deg2rad(zeta(2));
            x = deg2rad(zeta(3));
        otherwise
    end
     R_ZYX = [cos(z)*cos(y)...
        (cos(z)*sin(y)*sin(x))-(sin(z)*cos(x)) ...
        (cos(z)*sin(y)*cos(x))+(sin(z)*sin(x));
        sin(z)*cos(y) ...
        (sin(z)*sin(y)*sin(x))+(cos(z)*cos(x)) ...
        (sin(z)*sin(y)*cos(x))-(cos(z)*sin(x));
        -sin(y) cos(y)*sin(x) cos(y)*cos(x)];

end