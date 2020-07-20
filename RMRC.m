clc
clear
MS0 = eul2tform([0 0 0]);   % Prosanatolismos trapeziou os pros vasi
                            % Idios me auton tis vasis
PS0 = [500 0 100]';       % Thesi akmis trapeziou
MS0(1:3,4) = PS0;

MAS = eul2tform([0 0 0]);   % prosanatolismos A os pros trapezi
PAS = [0 -200 0]';           % thesi A os pros trapezi
MAS(1:3,4) = PAS;

MBS = eul2tform([0 0 0]);
PBS = [0 200 0]';          % kinoumaste stin dieuthinsi XS kata 40cm
MBS(1:3,4) = PBS;

MAT = eul2tform([0 0 0]);   % prosanatolismos ergaleiou os pros a kata ti sig.
MBT = eul2tform([0 0 0]);

MT6 = eul2tform([-pi -pi/2 0]); % prosanatolismos ergaleiou os pros 6
MT6(1:3,4) = [125 0 -(250+95)]';  % thesi syst. ergaleiou os pros 6

% euresi simeiou 6 otan to ergaleiou vrisketai stin ekkinisi A kai
% termatismo B
M60_A = MS0*MAS*inv(MAT)*inv(MT6); % pinakas gia lisi tou antistrofou
M60_B = MS0*MBS*inv(MBT)*inv(MT6);

fA = rotm2eul(M60_A(1:3,1:3));  % gonies euler
PA = M60_A(1:3,4);              % thesi 6 gia A
fB = rotm2eul(M60_B(1:3,1:3));
PB = M60_B(1:3,4);
disp('Inverse Kinematics problem for A')
[qA1,qA2,qA3,qA4,qA5,qA6] = invkin(PA(1),PA(2),PA(3),fA(1),fA(2),fA(3)) % gonies arthroseon gia A
disp('Forward Kinematics problem for A')
MA = forkin([qA1 qA2 qA3 qA4 qA5 qA6])                                  % pianakas met/smou gia A

reachA = sqrt(MA(1,4)^2+MA(2,4)^2+MA(3,4)^2);    % elegxos os pros oria xorou ergasias

disp('Inverse Kinematics problem for B')
[qB1,qB2,qB3,qB4,qB5,qB6] = invkin(PB(1),PB(2),PB(3),fB(1),fB(2),fB(3))
disp('Forward Kinematics problem for B')
MB = forkin([qB1 qB2 qB3 qB4 qB5 qB6])

reachB = sqrt(MA(1,4)^2+MA(2,4)^2+MA(3,4)^2);


%RMRC
ii = 0;
for n = [400 800]
    switch ii
        case 0
            col = 'b';
            label = 'Sample Points = 100';
        case 6
            col = 'r';
            label = 'Sample Points = 200';
        case 12
            col = 'g';
            label = 'Sample Points = 400';
        otherwise
    end
    dist = 400; % apostasi
    u = 50;
    dt = dist/u; % sinolikos xronos
    % time = linspace(0,dt,100);

    % Initialization
    q1 = [qA1 qA2 qA3 qA4 qA5 qA6]';
    jv = jacob(q1');
    M1 = M60_A;
    M7A = M1*MT6;
    M2 = zeros(4,4);
    u_lin = [0 50 0]; % p_dot (1:3)
    u_rot = [0 0 0]; % p_dot (4:6)
    u = [u_lin u_rot]';

    det_ = det(jv);

    th1 = q1(1);
    th2 = q1(2);
    th3 = q1(3);
    th4 = q1(4);
    th5 = q1(5);
    th6 = q1(6);

    xpos = M7A(1,4);
    ypos = M7A(2,4);
    zpos = M7A(3,4);
    
    t2 = dt/n;
    t1 = 0;
    M70_A = MA*MT6;
    M70_B = MB*MT6;
    dth = [];
    M70_2 = M70_A;
    
    while sum(sum(fix((M70_2(2,4) - M70_B(2,4))))) ~= 0
        q_dot = 180/pi * inv(jv)*u;   
        q2 = q1 + q_dot*(t2-t1);
        M2 = forkin(q2');
        M70_2 = M2*MT6;     % Thesi ergaleiou gia to neo set gonion

        q1 = q2;
        M2 = forkin(q2);
        jv = jacob(q2);

        t1 = t2;
        t2 = t2+dt/n;

        det_ = [det_ det(jv)];

        dth = [dth q_dot];      % Taxitites arthroseon
        dth1 = dth(1,:);
        dth2 = dth(2,:);
        dth3 = dth(3,:);
        dth4 = dth(4,:);
        dth5 = dth(5,:);
        dth6 = dth(6,:);

        th1 = [th1 q2(1)];      % Times gonion arthroseon
        th2 = [th2 q2(2)];
        th3 = [th3 q2(3)];
        th4 = [th4 q2(4)];
        th5 = [th5 q2(5)];
        th6 = [th6 q2(6)];

        xpos = [xpos M70_2(1,4)];   % Thesi ergaleiou ston xoro
        ypos = [ypos M70_2(2,4)];
        zpos = [zpos M70_2(3,4)];
    end

    figure(ii+1)
    subplot(2,3,1)
    plot(dth1, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_1}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,2)
    plot(dth2, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_2}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,3)
    plot(dth3, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_3}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,4)
    plot(dth4, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_4}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,5)
    plot(dth5, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_5}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,6)
    plot(dth6, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('$\dot{q_6}$','Interpreter','latex','Fontsize',16)
    sgtitle('Joint Velecities in [deg/s]')
    
    figure(ii+2)
    subplot(2,3,1)
    plot(th1, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_1}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,2)
    plot(th2, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_2}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,3)
    plot(th3, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_3}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,4)
    plot(th4, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_4}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,5)
    plot(th5, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_5}$','Interpreter','latex','Fontsize',16)
    subplot(2,3,6)
    plot(th6, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('${q_6}$','Interpreter','latex','Fontsize',16)
    sgtitle('Joint Angles in [deg]')

    figure(ii+3),plot(det_/10^8, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Determinant Value (x10^8)')
    title('Determinant of Jacobian')
 

    figure(ii+4)
    subplot(3,1,1)
    plot(xpos, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('X Position [mm]')
 

    subplot(3,1,2)
    plot(ypos, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Y Position [mm]')
 

    subplot(3,1,3)
    plot(zpos, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Z Position [mm]')
    sgtitle('Tool Positions')

 

    x = 500*ones(1,length(ypos));
    y = linspace(-200,200,length(xpos));
    z = 100*ones(1,length(zpos));

    errorx = xpos - x;
    errory = ypos - y;
    errorz = zpos - z;
    figure(ii+5)
    subplot(3,1,1)
    plot(errorx, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Error X [mm]')
 

    subplot(3,1,2)
    plot(errory, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Error Y [mm]')
 

    subplot(3,1,3)
    plot(errorz, 'Color', col, 'LineWidth', 1.5)
    xlabel('Sample Point')
    ylabel('Error Z [mm]')
    sgtitle('Errors')

    figure(ii+6)
    plot3(x, y, z, '-x', 'Color', 'r', 'LineWidth', 1.5)
    hold on
    plot3(xpos,ypos,zpos,'-x', 'Color', 'b', 'LineWidth', 1.5)
    xlabel('X position [mm]')
    ylabel('Y position [mm]')
    zlabel('Z position [mm]')
    title('Comparisson between Desired and Generated path')
    hold off
    grid on
    legend('Desired','Genarated')
    ii = ii+6;
end

for ii = 1:5
    L = findobj(ii,'type','line');
    copyobj(L,findobj(ii+6,'type','axes'));
end

close(figure(1),figure(2),figure(3),figure(4),figure(5))