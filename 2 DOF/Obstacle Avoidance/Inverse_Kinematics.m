
function IK = Inverse_Kinematics(X_ee, Y_ee)
    
    L1 = 5;
    L2 = 6;
    % Inverse Kinematics Calculation Start
    
    % End Effector State (R, Theta)
    R = sqrt((Y_ee*Y_ee) + (X_ee*X_ee));
    Theta = atan(Y_ee/(X_ee + 0.00001));
    
    % Intermediate Angles
    Beta = acos(((R*R)-(L1*L1)-(L2*L2))/(2*L1*L2));
    Alpha = asin((L2*sin(Beta))/R);
    
    % Final Joint Angles in Radians
    Final_Angle_1 = abs(Theta - Alpha);
    Final_Angle_2 = abs(pi - Beta);
    
    % Final Joint Angles in Degrees
    Final_Angle_1 = round(Final_Angle_1*(180/pi));
    Final_Angle_2 = round(Final_Angle_2*(180/pi));
    
    % Inverse Kinematics Calculation End

    IK = [Final_Angle_2, Final_Angle_1];

end