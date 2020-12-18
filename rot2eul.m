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
% This function computes the Jeometric Jacobian Jv.
function EulZYXAng = rot2eul(R)
    r11 = R(1,1); r12 = R(1,2); r21 = R(2,1); r31 = R(3,1); r32 = R(3,2);
    r33 = R(3,3);
    EulZYXAng(1) = atan2(r21,r11);
    EulZYXAng(2) = atan2(-r31,sqrt(r11^2+r21^2));
    EulZYXAng(3) = atan2(r32,r33);
end