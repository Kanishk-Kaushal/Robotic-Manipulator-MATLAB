% Program for Plotting a Configuration Space of a 2 DOF Robotic Manipulator

% Inputs -> Start

% Link Lengths
L1 = 5;
L2 = 5;

% Configuration Space | Workspace
Theta_Traj = linspace(0, 2*pi, 101);
Rad_Traj = linspace(abs(L1 - L2), (L1 + L2), 101);
Final_y = [];
Final_x = [];

for j = 1 : length(Rad_Traj)
    for k = 1 : length(Theta_Traj)

        Final_X = Rad_Traj(k)*cos(Theta_Traj(j));
        Final_x(j*k) = Final_X;

        Final_Y = Rad_Traj(k).*sin(Theta_Traj(j));
        Final_y(j*k) = Final_Y;

    end
end

% Inputs -> End

% Arrays that will store End Effector Positions
X_Arr = [];
Y_Arr = [];

% Arrays that will store Joint Angles
JA_1 = [];
JA_2 = [];

% Counter and Loop Initialization
counter = 1;

for i = 1 : length(Theta_Traj)*length(Rad_Traj)

    % Desired End Effector Position Beyond Workspace -> Start

    % Maximum Extened Arm Length
    Arm_Radius = L1 + L2;

    % Arm Length Differnece
    Arm_Diff = abs(L1 - L2);

    % End Effector Distance from Origin
    D = sqrt((Final_y.*Final_y) + (Final_x.*Final_x));

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

    % Desired End Effector Position Beyond Workspace -> End

    % Plotting the Workspace Start

    Theta_Circle = (0:pi/50:2*pi);

    % Plotting the Outer Workspace Boundaries
    X_outer_WS = (Arm_Radius*cos(Theta_Circle));
    Y_outer_WS = (Arm_Radius*sin(Theta_Circle));

    % Plotting the Inner Workspace Boundaries
    X_inner_WS = (Arm_Diff*cos(Theta_Circle));
    Y_inner_WS = (Arm_Diff*sin(Theta_Circle));

    % Plotting the Workspace End

    % Inverse Kinematics Calculation Start

    % End Effector State (R, Theta)
    R = sqrt((Final_y.^2) + (Final_x.^2));
    Theta = atan(Final_y./(Final_x + 0.00001));
    % 0.000001 is added to avoid Undefined Cartesian Points

    % Intermediate Angles
    Beta = real(acos(((L1*L1) + (L2*L2) - (R.*R))/(2*L1*L2)));
    Alpha = real(acos(((L1*L1) + (R.*R) - (L2*L2))/(2*L1*R)));

    % 2nd Quadrant End Effector Position

    if (Final_x(i) < 0)

        Theta = Theta + pi;
        Alpha = (1)*Alpha;
        Beta = (1)*Beta;

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

    % Inverse Kinematics Calculation End

    % Array of Joint Angles
    Theta_1 = Final_Angle_1(i);
    Theta_2 = Final_Angle_2(i);

    % Forward Kinematics -> Start

    % Initial points of Link 1
    X0 = 0;
    Y0 = 0;

    % Final points of Link 1
    X1 = L1*cosd(Theta_1);
    Y1 = L1*sind(Theta_1);

    % Final points of Link 2
    X2 = X1 + L2*cosd(Theta_1 + Theta_2);
    Y2 = Y1 + L2*sind(Theta_1 + Theta_2);

    % Forward Kinematics -> End

    % Storing End Effector Positions to Plot Trajectory
    X_Arr(i) = X2;
    Y_Arr(i) = Y2;

    % Storing Joint Angles For Configuration Space | Scaled by factor of 15
    JA_1 = Theta_1/15;
    JA_2 = (Theta_2/15)

    % Storing Initial End Effector Coordinates for Plotting Trajectory
    if (counter == 1)

        Eef_X_init = X2;
        Eef_Y_init = Y2;

    end

    % Assigning the Parameters
    txt1 = ['T1 = ', num2str(Theta_1), ' Deg'];
    txt2 = ['T2 = ', num2str(Theta_2), ' Deg'];
    txtend = ['X2 = ', num2str(X2), ', ', 'Y2 = ', num2str(Y2)];

    % Plot Configuration Space
    hold on
    plot(JA_1, JA_2, '.k', 'MarkerSize', 69)
    hold off

    xlabel('Joint Angle 1 (Deg/15)')
    ylabel('Joint Angle 2 (Deg/15)')
    title('Configuration Space of a 2 DOF Robotic Manipulator');

    grid on
    axis([-19 19 -19 19])

    M(counter) = getframe(gcf);
    counter = counter + 1;



end
