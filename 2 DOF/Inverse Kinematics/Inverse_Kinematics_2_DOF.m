% Program for Inverse Kinematics on a 2 DOF Robotic Manipulator

% Input Start 

% Link Lengths
L1 = 1;
L2 = 1;

% Final End Effector Position
Final_x = -1;
Final_y = 0.5;

% Input End

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

% Plotting the Workspace Start

Theta_Circle = 0:pi/50:2*pi;

% Plotting the Outer Workspace Boundaries
X_outer_WS = (Arm_Radius*cos(Theta_Circle));
Y_outer_WS = (Arm_Radius*sin(Theta_Circle));

% Plotting the Inner Workspace Boundaries
X_inner_WS = (Arm_Diff*cos(Theta_Circle));
Y_inner_WS = (Arm_Diff*sin(Theta_Circle));

% Plotting the Workspace End

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

% Inverse Kinematics Calculation End

% Angle with the Horizontal (Degrees)
A1 = linspace(0, (Final_Angle_1), 30);
A2 = linspace(0, (Final_Angle_2), 30);

% Forward Kinematics:

% Counter and Loop Initialization
counter = 1;

% Arrays that will stor End Effector Positions
X_Arr = [];
Y_Arr = [];

for i = 1 : length(A1)

    Theta_1 = A1(i);
    Theta_2 = A2(i);
    
    % Coordinates:

    % Initial points of Link 1
    X0 = 0;
    Y0 = 0;

    % Final points of Link 1 
    X1 = L1*cosd(Theta_1);
    Y1 = L1*sind(Theta_1);
    
    % Final points of Link 2
    X2 = X1 + L2*cosd(Theta_1 + Theta_2);
    Y2 = Y1 + L2*sind(Theta_1 + Theta_2);
    
    % Storing End Effector POsitions to Plot Trajectory
    X_Arr(i) = X2;
    Y_Arr(i) = Y2;

    % Storing Initial End Effector Coordinates for Plotting Trajectory
    if (counter == 1)
        
        Eef_X_init = X2;
        Eef_Y_init = Y2;

    end

    % Assigning the Parameters
    txt1 = ['T1 = ', num2str(Theta_1), ' Deg'];
    txt2 = ['T2 = ', num2str(Theta_2), ' Deg'];
    txtend = ['X2 = ', num2str(X2), ', ', 'Y2 = ', num2str(Y2)];

    % Plot
    plot([X0 X1], [Y0 Y1], [X1 X2], [Y1 Y2], 'linewidth', 5)

    hold on

    % Plots Displacement
    plot([Eef_X_init X2], [Eef_Y_init Y2], '-*')
    
    % Plots Trajectory
    plot(X_Arr, Y_Arr, 'o')

    plot(X_inner_WS, Y_inner_WS);
    plot(X_outer_WS, Y_outer_WS);

    % Plots Workspace
    fill([X_inner_WS flip(X_outer_WS)],[Y_inner_WS flip(Y_outer_WS)],'g')

    hold off
    
    % Increases Transparency of Workspace
    alpha(0.1)

    xlabel('X-Axis (m)')
    ylabel('Y-Axis (m)')
    title('Inverse Kinematics on a 2 DOF Robotic Manipulator');
    text(X0, Y0, txt1, 'VerticalAlignment', 'top')
    text(X1, Y1, txt2, 'VerticalAlignment', 'top')
    text(X2, Y2, txtend, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom')
    text(0.5*(X0 + X1), 0.5*(Y0 + Y1), '  Link 1')
    text(0.5*(X1 + X2), 0.5*(Y1 + Y2), '  Link 2')
    text(((-1)*Final_x), 0, 'Workspace')
    grid on

    axis([-3 3 -3 3])

    M(counter) = getframe(gcf);
    counter = counter + 1;
    
end
