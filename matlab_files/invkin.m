% Alexandros Dimas
% University of Patras
% Department of Mechanical Engineering and Aeronautics
% Robotics Group
% Spring 2020

%{

Convention 1: MBA refers to pose of frame {B} as seen by {A}. Similar
notation is used to describe relative position and orientation of two
frames.

For equations used see:
John J. Craig. 2004. Introduction to Robotics: Mechanics and Control
(3rd. ed.). Addison-Wesley Longman Pvelocityblishing Co., Inc., USA,
p. 117-121

There are 8 possible solution. The script checkes every single one if
it is outside robot's reach and returns the last possible.

%}

function [theta1,theta2,theta3,theta4,theta5,theta6] = invkin(Px,Py,Pz,fz,fy,fx)
        [a,l,d] = get_dh();
        nogo = 0;
        % Because the sqrt term in theta3 can be + or - we run through
        % all possible combinations (i = 2) and take the first combination that
        % satisfies the joint angle constraints.
        sols = [];
        while nogo == 0
            for i = 1:4
                if i == 1
                    sign1 = 1;
                    sign2 = 1;
                elseif i == 2
                    sign1 = 1;
                    sign2 = -1;
                elseif i == 3
                    sign1 = -1;
                    sign2 = 1;
                else
                    sign1 = -1;
                    sign2 = -1;
                end
                % Here "a" refers to link length not twist
                a1 = l(2);
                a2 = l(3);
                a3 = l(4);
                
                d3 = d(3);
                d4 = d(4);
                rho = sqrt(Px^2+Py^2);
                phi = atan2(Py,Px);
                theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));
                c1 = cos(theta1);
                s1 = sin(theta1);
                
                K = (Px^2+Py^2+Pz^2+a1^2-(2*Px*a1)/c1-a2^2-a3^2-d4^2)/(2*a2);
                theta3 = (atan2(a3,d4)-atan2(K,sign2*sqrt(a3^2+d4^2-K^2)));
                c3 = cos(theta3);
                s3 = sin(theta3);
                
                theta23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py-a1)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py-a1));
                theta2 = (theta23 - theta3);
                c2 = cos(theta2);
                s2 = sin(theta2);

                R = eul2rot([fz fy fx]);
                s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py-a1)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py-a1)^2);
                c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py-a1))/(Pz^2+(c1*Px+s1*Py-a1)^2);
                r13 = R(1,3);
                r23 = R(2,3);
                r33 = R(3,3);
                theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);
                c4 = cos(theta4);
                s4 = sin(theta4);
                
                r11 = R(1,1);
                r21 = R(2,1);
                r31 = R(3,1);
                s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
                c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
                theta5 = atan2(s5,c5);

                s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
                c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
                theta6 = atan2(s6,c6);

                theta1 = theta1*180/pi;
                theta2 = theta2*180/pi;
                theta3 = theta3*180/pi;
                theta4 = theta4*180/pi;
                theta5 = theta5*180/pi;
                theta6 = theta6*180/pi;
                sols = [sols;theta1 theta2 theta3 theta4 theta5 theta6;theta1 theta2 theta3 theta4+180 -theta5 theta6+180]; % Complete solution matrix
                
                % Solution check
                % Last possible solution chosen
                if theta1<=185 && theta1>=-185 && (theta2<=65 && theta2>=-185) && theta3<=175 && theta3>=-138 && theta4<=350 && theta4>=-350 && theta5<=130 && theta5>=-130 && theta6<=350 && theta6>=-350
                    nogo = 1;
                    break
                end
                if i == 4 && nogo == 0
                    h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                    waitfor(h);
                    nogo = 1;
                    break
                end
            end
         end
     end