function Tform = eul2trm(x)
    % Input must be in raw vector form
    if size(x,2) == 1
        x = x';
    end
    R = eul2rot(x); % Euler rotation matrix
    Tform = [R zeros(3,1);
        0 0 0 1];
end