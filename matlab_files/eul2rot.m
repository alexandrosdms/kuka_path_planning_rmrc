% Alexandros Dimas
% University of Patras
% Department of Mechanical Engineering and Aeronautics
% Robotics Group
% Spring 2020

% Euler to Rotation Matrix

%{

Convention 1: MBA refers to pose of frame {B} as seen by {A}. Similar
notation is used to describe relative position and orientation of two
frames.

%}

function RotationMatrix = eul2rot(eulerZYXangles)
    % Input must be in raw vector form
    if size(eulerZYXangles,2) == 1
        eulerZYXangles = eulerZYXangles';
    end
    x1 = eulerZYXangles(1); x2 = eulerZYXangles(2); x3 = eulerZYXangles(3);
    s1 = sin(x1);
    c1 = cos(x1);
    s2 = sin(x2);
    c2 = cos(x2);
    s3 = sin(x3);
    c3 = cos(x3);
    
    RotationMatrix = [c1*c2 c1*s2*s3-s1*c3 c1*s2*c3+s1*s3;
        s1*c2 s1*s2*s3+c1*c3 s1*s2*c3 - c1*s3;
        -s2 c2*s3 c2*c3];
end