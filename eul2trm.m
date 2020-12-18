% Alexandros Dimas
% University of Patras
% Department of Mechanical Engineering and Aeronautics
% Robotics Group
% Spring 2020
%{

Convention 1: MBA refers to pose of frame {B} as seen by {A}. Similar
notation is used to describe relative position and orientation of two
frames.

%}
function HomogeneousTransformation = eul2trm(eulerZYXangles)
    % Input must be in raw vector form
    if size(eulerZYXangles,2) == 1
        eulerZYXangles = eulerZYXangles';
    end
    R = eul2rot(eulerZYXangles); % Euler rotation matrix
    HomogeneousTransformation = [R zeros(3,1);
        0 0 0 1];
end