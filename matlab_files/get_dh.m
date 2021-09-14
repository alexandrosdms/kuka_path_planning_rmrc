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

function [a,l,d] = get_dh()
    a = [0 -pi/2 0 -pi/2 pi/2 -pi/2]; % link twist vector in radians
    l = [0 160 780 150 0 0]; % link length vector in mm
    d = [0 0 0 655 0 0]; % link offser vector in mm
end

