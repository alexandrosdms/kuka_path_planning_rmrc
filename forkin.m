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

function M60 = forkin(x)
    % Get DH parameters
    [a,l,d] = get_dh();
    th = deg2rad(x);

    numDegreesFreedom = 1:size(th,2);
    Mt = eye(4); % Initialization
    for i = numDegreesFreedom
        M = [cos(th(i)) -sin(th(i)) 0 l(i);
            sin(th(i))*cos(a(i)) cos(th(i))*cos(a(i)) -sin(a(i)) -sin(a(i))*d(i);
            sin(th(i))*sin(a(i)) cos(th(i))*sin(a(i)) cos(a(i)) cos(a(i))*d(i);
            0 0 0 1]; % Pose of {i-1} joint frame with respect to {i}
        % For i = 1, M = M10

        Mt = Mt*M; % Pose of {i} with respect to base {0}
    end

    M60 = Mt;
%     p = M60(1:3,4); % TCP Position
%     f = rot2eul(M60(1:3,1:3));
end