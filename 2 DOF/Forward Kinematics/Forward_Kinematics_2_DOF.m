% Program for Forward Kinematics on a 2 DOF Robotic Manipulator

% Input Start 

% Link Lengths
L1 = 1;
L2 = 0.4;

% Final Angles
Final_Angle_1 = 90;
Final_Angle_2 = 135;

% Input End

% Angle with the horizontal (Degrees)
A1 = linspace(0, Final_Angle_1, 10);
A2 = linspace(0, Final_Angle_2, 30);

% Solution:

% Counter and Loop Initialization
counter = 1;

for i = 1 : length(A1)

    Theta_1 = A1(i);

    for j = 1 : length(A2)

        Theta_2 = A2(j);
        
        % Coordinates:

        % Initial points of Link 1
        X0 = 0;
        Y0 = 0;

        % Final points of Link 1 
        X1 = L1*cosd(Theta_1);
        Y1 = L1*sind(Theta_1);
        
        % FInal points of Link 2
        X2 = X1 + L2*cosd(Theta_2);
        Y2 = Y1 + L2*sind(Theta_2);

        % Assigning the Parameters
        txt1 = ['T1 = ', num2str(Theta_1), ' Deg'];
        txt2 = ['T2 = ', num2str(Theta_2), ' Deg'];
        txtend = ['X2 = ', num2str(X2), ', ', 'Y2 = ', num2str(Y2)];

        % Plot
        plot([X0 X1], [Y0 Y1], [X1 X2], [Y1 Y2], 'linewidth', 5)
        xlabel('X-Axis (m)')
        ylabel('Y-Axis (m)')
        title('Forward Kinematics on a 2 DOF Robotic Manipulator');
        text(X0, Y0, txt1, 'VerticalAlignment', 'top')
        text(X1, Y1, txt2, 'VerticalAlignment', 'top')
        text(X2, Y2, txtend, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom')
        text(0.5*(X0 + X1), 0.5*(Y0 + Y1), '  Link 1')
        text(0.5*(X1 + X2), 0.5*(Y1 + Y2), '  Link 2')
        grid on
        axis([-1 2 -0.5 2])
        pause(0.3)

        M(counter) = getframe(gcf);
        counter = counter + 1;
    end
end

% Creating the Animation
movie(M);
videofile = VideoWriter('Forward Kinematics on a 2 DOF Robotic Manipulator.avi', 'Uncompressed AVI');
open(videofile);
writeVideo(videofile, M);
close(videofile);
