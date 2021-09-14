% Alexandros Dimas
% University of Patras
% Department of Mechanical Engineering and Aeronautics
% Robotics Group
% Spring 2020
%{

A couple of conventions:

Convention 1: MBA refers to pose of frame {B} as seen by {A}. Similar
notation is used to describe relative position and orientation of two
frames.

Covention 2: Table frame {S} is attaches to the geometric center of a table
of leangth of 500 mm at least. The orientation of {S} is the same as that of 
the base frame{0} and it is translated 500mm in the x-direction and
100mm in the z-direction with respect to the same frame (See MS0 and PS0)

%}

clc, clear, close('all');

%------------------------- Attaching Frames ------------------------------%
MS0 = eul2trm([0 0 0]);
PS0 = [500 0 100]';
MS0(1:3,4) = PS0;

% Frames attached to start and end of line path
MAS = eul2trm([0 0 0]); %Start of line path
PAS = [0 -200 0]';
MAS(1:3,4) = PAS;

MBS = eul2trm([0 0 0]);
PBS = [0 200 0]';
MBS(1:3,4) = PBS;

% Desired pose of frames {A} and {B} with respect to tool {T}. We choose
% coincedence to be the relationship between {A} and {T} and {B} and {T}
% when the tool approaches either one of them
MAT = eul2trm([0 0 0]); 
MBT = eul2trm([0 0 0]);

% Pose of tool with respect to wrist frame {6} - predetermined
MT6 = eul2trm([pi -pi/2 0]);
MT6(1:3,4) = [-125 0 (250+153)]';

%{
Knowing the above we compute the wrist's frame pose when tool frame has
the desired pose relative to {A} and {B} as was stated by MAT and MBT
respectively.

See John J. Craig. 2004. Introduction to Robotics: Mechanics and Control
(3rd. ed.). Addison-Wesley Longman Pvelocityblishing Co., Inc., USA,
p. 125-126
%}

M60_A = MS0*MAS/MAT/MT6; % Pose of {6} relative to {0} when at {A}
M60_B = MS0*MBS/MBT/MT6;

%------------------- Inverse & Forward kinematics problems ---------------%
fA = rot2eul(M60_A(1:3,1:3)); % Evelocityler ZYX angles
PA = M60_A(1:3,4); % Relative displacement of {A}
fB = rot2eul(M60_B(1:3,1:3));
PB = M60_B(1:3,4);
disp('Inverse Kinematics problem for A')
[qA1,qA2,qA3,qA4,qA5,qA6] = invkin(PA(1),PA(2),PA(3),fA(1),fA(2),fA(3))
disp('Forward Kinematics problem for A')
MA = forkin([qA1 qA2 qA3 qA4 qA5 qA6])

disp('Inverse Kinematics problem for B')
[qB1,qB2,qB3,qB4,qB5,qB6] = invkin(PB(1),PB(2),PB(3),fB(1),fB(2),fB(3))
disp('Forward Kinematics problem for B')
MB = forkin([qB1 qB2 qB3 qB4 qB5 qB6])

% For debugging, velocity if out of bounds not checked***!!!!
% reachA = sqrt(MA(1,4)^2+MA(2,4)^2+MA(3,4)^2);
% reachB = sqrt(MA(1,4)^2+MA(2,4)^2+MA(3,4)^2);


%--------------------------- RMRC Algorithm ------------------------------%
ii = 0;
for n = [400 800] %two sample sizes for comparison
    % Switching color of graphs for each sample size
    switch ii
        case 0
            col = 'b';
            label = 'Sample Points = 400';
            jj = 1;
        case 6
            col = 'r';
            label = 'Sample Points = 800';
            jj = 2;
        otherwise
    end
    distance = 400; %in mm
    velocity = 50; %in mm/s
    durationMovement = distance/velocity;
    
    % Initialization of parameters
    q1 = [qA1 qA2 qA3 qA4 qA5 qA6]'; % Joint angles
    jv = jacob(q1');
    M1 = M60_A;
    M7A = M1*MT6;
    M2 = zeros(4,4);
    linearVelocity = [0 50 0]; % p_dot (1:3)
    angularVelocity = [0 0 0]; % p_dot (4:6)
    velocity = [linearVelocity angularVelocity]';

    detJacob = det(jv);

    th1 = q1(1);
    th2 = q1(2);
    th3 = q1(3);
    th4 = q1(4);
    th5 = q1(5);
    th6 = q1(6);

    xPosition = M7A(1,4);
    yPosition = M7A(2,4);
    zPosition = M7A(3,4);
    
    t1 = 0;
    t2 = durationMovement/n;
    
    M70_A = MA*MT6;
    M70_B = MB*MT6;
    timeDerJointAngles = [];
    M70_2 = M70_A;
    
    condition = (sum(sum(fix((M70_2 - M70_B)))) ~= 0);
    %{ 
        As long as the tool frame at each iteration does not coincide with
        frame {B} at the path end the above condition will be true
    %}
    while condition
        q_dot = (180/pi)*eye(6)/jv*velocity; % Time derivative approximation  
        q2 = q1 + q_dot*(t2-t1); % Numeric integration
        M2 = forkin(q2'); % New pose
        M70_2 = M2*MT6; % New pose of tool

        % Initialization for next iteration
        q1 = q2;
        jv = jacob(q2);
        t1 = t2;
        t2 = t2+durationMovement/n;
        condition = (sum(sum(fix((M70_2 - M70_B)))) ~= 0);
        
        % Saving parameters to plot
        detJacob = [detJacob det(jv)];

        timeDerJointAngles = [timeDerJointAngles q_dot];      % Taxitites arthroseon
        timeDerJointAngles1 = timeDerJointAngles(1,:);
        timeDerJointAngles2 = timeDerJointAngles(2,:);
        timeDerJointAngles3 = timeDerJointAngles(3,:);
        timeDerJointAngles4 = timeDerJointAngles(4,:);
        timeDerJointAngles5 = timeDerJointAngles(5,:);
        timeDerJointAngles6 = timeDerJointAngles(6,:);

        th1 = [th1 q2(1)];      % Times gonion arthroseon
        th2 = [th2 q2(2)];
        th3 = [th3 q2(3)];
        th4 = [th4 q2(4)];
        th5 = [th5 q2(5)];
        th6 = [th6 q2(6)];

        xPosition = [xPosition M70_2(1,4)];   % Thesi ergaleiovelocity ston xoro
        yPosition = [yPosition M70_2(2,4)];
        zPosition = [zPosition M70_2(3,4)];
    end
    
    % Plot graphs
    set(0,'defaulttextinterpreter','latex')
    figure(ii+1)
    subplot(2,3,1)
    plot(timeDerJointAngles1, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_1} [{}^o/s]$', 'Fontsize',14)
    subplot(2,3,2)
    plot(timeDerJointAngles2, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_2} [{}^o/s]$', 'Fontsize',14)
    subplot(2,3,3)
    plot(timeDerJointAngles3, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_3} [{}^o/s]$', 'Fontsize',14)
    subplot(2,3,4)
    plot(timeDerJointAngles4, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_4} [{}^o/s]$', 'Fontsize',14)
    subplot(2,3,5)
    plot(timeDerJointAngles5, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_5} [{}^o/s]$', 'Fontsize',14)
    subplot(2,3,6)
    plot(timeDerJointAngles6, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$\dot{q_6} [{}^o/s]$', 'Fontsize',14)
    sgtitle('Joint Velecities','FontSize',16)
    
    figure(ii+2)
    subplot(2,3,1)
    plot(th1, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_1} [{}^o]$', 'Fontsize',14)
    subplot(2,3,2)
    plot(th2, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_2} [{}^o]$', 'Fontsize',14)
    subplot(2,3,3)
    plot(th3, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_3} [{}^o]$', 'Fontsize',14)
    subplot(2,3,4)
    plot(th4, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_4} [{}^o]$', 'Fontsize',14)
    subplot(2,3,5)
    plot(th5, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_5} [{}^o]$', 'Fontsize',14)
    subplot(2,3,6)
    plot(th6, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('${q_6} [{}^o]$', 'Fontsize',14)
    sgtitle('Joint Angles','FontSize',16)

    figure(ii+3),plot(detJacob/10^8, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('$Determinant Valvelocitye (\times 10^8)$', 'Fontsize',12)
    title('Determinant of Jacobian','FontSize',16)
 

    figure(ii+4)
    subplot(3,1,1)
    plot(xPosition, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('X [mm]', 'Fontsize',12)
 

    subplot(3,1,2)
    plot(yPosition, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('Yn[mm]', 'Fontsize',12)
 

    subplot(3,1,3)
    plot(zPosition, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('Z [mm]', 'Fontsize',12)
    sgtitle('Tool Position','FontSize',16)

 

    x = 500*ones(1,length(yPosition));
    y = linspace(-200,200,length(xPosition));
    z = 100*ones(1,length(zPosition));

    errorx = xPosition - x;
    errory = yPosition - y;
    errorz = zPosition - z;
    figure(ii+5)
    subplot(3,1,1)
    plot(errorx, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('X [mm]', 'Fontsize',12)
 

    subplot(3,1,2)
    plot(errory, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('Y [mm]', 'Fontsize',12)
 

    subplot(3,1,3)
    plot(errorz, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point', 'Fontsize',12)
    ylabel('Z [mm]', 'Fontsize',12)
    sgtitle('Error Between Desired and Generated Path','FontSize',14)

    figure(ii+6)
    plot3(x, y, z, '-x', 'Color', 'r', 'LineWidth', 1.5)
    hold on
    plot3(xPosition,yPosition,zPosition,'-x', 'Color', 'b', 'LineWidth', 1.5)
    xlabel('X position [mm]', 'Fontsize',12)
    ylabel('Y position [mm]', 'Fontsize',12)
    zlabel('Z position [mm]', 'Fontsize',12)
    title('Comparisson between Desired and Generated path','FontSize',16)
    hold off
    grid on
    legend('Desired','Genarated')
    ii = ii+6;
end

% Combine graphs for comparison
for ii = 1:5
    L = findobj(ii,'type','line');
    copyobj(L,findobj(ii+6,'type','axes'));
end
% Show graphs for sample size n = 400 first
for k = 5:-1:1
    switch(k)
        case {1,2}
            l = 2; m = 3;
        case 3
            l = 1; m = 1;
        case {4,5}
            l = 3; m = 1;
        otherwise
    end
    j = 1:l*m;
    for j = j
        figure(k),subplot(l,m,j),xlim([0 400]),xticks(0:50:400),xticklabels(0:8),xlabel('Time [s]', 'Fontsize',12);
    end
end