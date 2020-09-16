function RotMat = eul2rot(x)
    % Input must be in raw vector form
    if size(x,2) == 1
        x = x'
    end
    x1 = x(1); x2 = x(2); x3 = x(3);
    s1 = sin(x1);
    c1 = cos(x1);
    s2 = sin(x2);
    c2 = cos(x2);
    s3 = sin(x3);
    c3 = cos(x3);
    
    RotMat = [c1*c2 c1*s2*s3-s1*c3 c1*s2*c3+s1*s3;
        s1*c2 s1*s2*s3+c1*c3 s1*s2*c3 - c1*s3;
        -s2 c2*s3 c2*c3];
end