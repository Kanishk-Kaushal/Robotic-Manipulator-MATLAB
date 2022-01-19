function IK = Inverse_Kinematics(Final_x, Final_y)
    
    % Refer to TwoLinkRobot.m for Dimensions
    L1 = 5;
    L2 = 5;
    
    % Desired End Effector Position Beyond Workspace Start

    % Maximum Extened Arm Length
    Arm_Radius = L1 + L2;
    
    % Arm Length Differnece 
    Arm_Diff = abs(L1 - L2);
    
    % End Effector Distance from Origin
    D = sqrt((Final_y*Final_y) + (Final_x*Final_x));
    
    % Outer Workspace Boundary
    if (D > Arm_Radius)
        Final_x = (Arm_Radius)*(Final_x)/(D);
        Final_y = (Arm_Radius)*(Final_y)/(D);
    end
    
    % Inner Workspace Boundary
    if(D < Arm_Diff)
        Final_x = (Arm_Diff)*(Final_x)/(D);
        Final_y = (Arm_Diff)*(Final_y)/(D);
    end

    % Desired End Effector Position Beyond Workspace End

    % Inverse Kinematics Calculation Start

    % End Effector State (R, Theta)
    R = sqrt((Final_y*Final_y) + (Final_x*Final_x));
    Theta = atan(Final_y/(Final_x + 0.000001));
    % 0.000001 is added to avoid Undefined Cartesian Points
    
    % Intermediate Angles
    Beta = acos(((L1*L1) + (L2*L2) - (R*R))/(2*L1*L2));
    Alpha = acos(((L1*L1) + (R*R) - (L2*L2))/(2*L1*R));
    
    % 2nd Quadrant End Effector Position
    if (Final_x < 0)
    
        Theta = Theta + pi;
        Beta = (-1)*Beta;
        Alpha = (-1)*Alpha;
    
    end
    
    % Final Joint Angles in Radians
    
    % Elbow UP Configuration
    Final_Angle_1 = (Theta + Alpha);
    Final_Angle_2 = (Beta - pi);
    
    % Elbow DOWN Configuration
    % Final_Angle_1 = (Theta - Alpha);
    % Final_Angle_2 = pi - Beta;
    
    % Final Joint Angles in Degrees
    Final_Angle_1 = Final_Angle_1*(180/pi);
    Final_Angle_2 = Final_Angle_2*(180/pi);

    if(Final_Angle_1 <= 0)
        Final_Angle_1 = 360 + Final_Angle_1;
    end

     if(Final_Angle_2 <= 0)
        Final_Angle_2 = 360 + Final_Angle_2;
    end
    
    % Inverse Kinematics Calculation End

    IK = [round(Final_Angle_1 ), abs(round(Final_Angle_2))];

end