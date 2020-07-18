function [theta1,theta2,theta3,theta4,theta5,theta6] = invkin(Px,Py,Pz,fz,fy,fx)
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
        sign1 = 1;
        sign3 = 1;
        nogo = 0;
        noplot = 0;
        % Because the sqrt term in theta3 can be + or - we run through
        % all possible combinations (i = 2) and take the first combination that
        % satisfies the joint angle constraints.
%         while nogo == 0;
%             for i = 1:1:4
%                 if i == 1
%                     sign1 = 1;
%                     sign3 = 1;
%                 elseif i == 2
%                     sign1 = 1;
%                     sign3 = -1;
%                 elseif i == 3
%                     sign1 = -1;
%                     sign3 = 1;
%                 else
%                     sign1 = -1;
%                     sign3 = -1;
%                 end
                a1 = 150;
                a2 = 590;
                a3 = 130;
                d4 = 647.07;
                rho = sqrt(Px^2+Py^2);
                phi = atan2(Py,Px);
                theta1 = atan2(Py,Px);
                c1 = cos(theta1);
                s1 = sin(theta1);
                K = (Px^2+Py^2+Pz^2+a1^2-(2*Px*a1)/c1-a2^2-a3^2-d4^2)/(2*a2);
                theta3 = (atan2(a3,d4)-atan2(K,sign3*sqrt(a3^2+d4^2-K^2)));
                c3 = cos(theta3);
                s3 = sin(theta3);
                t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py-a1)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py-a1));
                theta2 = (t23 - theta3);
                c2 = cos(theta2);
                s2 = sin(theta2);

                R = eul2rotm([fz fy fx]);
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
%                 if theta2>=360-115 && theta2<=360-35
%                     theta2 = 360-theta2;
%                 end
%                 if theta1<=180 && theta1>=-180 && (theta2<=-115 && theta2>=-115) && theta3<=-20 && theta3>=180 && theta4<=210 && theta4>=-210 && theta5<=130 && theta5>=-130 && theta6<=2700 && theta6>=-2700
%                     nogo = 1;
%                     break
%                 end
%                 if i == 4 && nogo == 0
%                     h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
%                     waitfor(h);
%                     nogo = 1;
%                     noplot = 1;
%                     break
%                 end
%             end
%          end
     end