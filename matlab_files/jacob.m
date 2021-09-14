% Alexandros Dimas
% University of Patras
% Department of Mechanical Engineering and Aeronautics
% Robotics Group
% Spring 2020

% This function computes the Jeometric Jacobian Jv

%{

Convention 1: MBA refers to pose of frame {B} as seen by {A}. Similar
notation is used to describe relative position and orientation of two
frames.

%}

function [jv] = jacob(jointAngles)
    if size(jointAngles,1) ~= 1
        jointAngles = jointAngles';
    end
    [a,l,d] = get_dh();
    th = deg2rad(jointAngles);
    
    Mt = eye(4);
    numDegreesFreedom = 1:size(jointAngles,2);
    % Getting tool position relative to bese "p70".
    for i = numDegreesFreedom
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1]; % Pose of {i-1} joint frame with respect to {i}

        Mt = Mt*M;
    end

    M60 = Mt;
    M76 = eul2trm([pi -pi/2 0]);
    M76(1:3,4) = [-125 0 (250+153)]';
    
    M70 = M60*M76;
    p70 = M70(1:3,4);
    
    % Computing the Jeometric Jocobian
    jvi = [];
    Mi = eye(4);
    for i = 1:6
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1];
        
        Mi = Mi*M; % Pose of {i} with respect to base {0}
        ki = Mi(1:3,3); % z-axis unit vection with respect to base {0}
        Pi = Mi(1:3,4); % relative position of {i} with respect to base {0}
        jvi_0 = [cross(ki,p70-Pi);ki]; % Jeometric Jacobian column i
        jvi = [jvi jvi_0];
    end
    jv = jvi;
end