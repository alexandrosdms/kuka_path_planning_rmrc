function EulAng = rot2eul(R)
    r11 = R(1,1); r12 = R(1,2); r21 = R(2,1); r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    EulAng(1) = atan2(r21,r11);
    EulAng(2) = atan2(-r31,sqrt(r11^2+r21^2));
    EulAng(3) = atan2(r32,r33);
end