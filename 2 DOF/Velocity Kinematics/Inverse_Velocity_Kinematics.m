% Inverse Velocity Kinematics for a 2R Planar Arm

% Inputs -> Start

% Link Lengths
L1 = 1;
L2 = 1;
Link_Lengths = [L1 L2];

% Initial End-Effector Configuration
init_eef_pos = [1 -1];
Theta = (IK(Link_Lengths, init_eef_pos));

curr_eef_pos = init_eef_pos;

% Desired End-Effector Configuration
des_eef_pos = [1 1];

% Inputs -> End

% Velocity Kinematics Timing Parameters
t = 10;
dt = 0.1;

% Velocity Vector
v = (des_eef_pos - curr_eef_pos)./t;

% Displacement Vector
r = sqrt((des_eef_pos(1) - curr_eef_pos(1))^2 + (des_eef_pos(2) - curr_eef_pos(2))^2);

% Distance
dist = 0;

i = 1;

while (((r - dist) > 0.001) && (i <= 1000))
    
    % Builds Jacobian Matrix
    Jacob = Jac(Theta, Link_Lengths);
    % Determinant
    det_Jacob = det(Jacob);
    % Pseudo-Inverse
    Jacob_pinv = pinv(Jacob);
    % Angular Velocities of Joints
    Theta1dot = (Jacob_pinv(1, 1)*v(1) + Jacob_pinv(1, 2)*v(2));
    Theta2dot = (Jacob_pinv(2, 1)*v(1) + Jacob_pinv(2, 2)*v(2));
    Thetadot = ([Theta1dot Theta2dot]);
    % Update Joint Angles
    Theta = Theta + (Thetadot.*dt);
    Theta_Deg = rad2deg(Theta);
    % Calculate Current End Effector Configuration
    curr_eef_pos = FK(Link_Lengths, Theta);
    % Update Distance
    dist = sqrt((curr_eef_pos(1) - init_eef_pos(1))^2 + (curr_eef_pos(2) - init_eef_pos(2))^2);
    % Update Velocity Vector
    v = (des_eef_pos - curr_eef_pos)/t;
    %vel(i) = v(1);
    i = i + 1;
  
 end


% Function to build Jacobian Matrix
function Jacobian = Jac(T, L)
    
    J11 = -L(1)*sin(T(1)) - L(2)*sin(T(1) + T(2));
    J12 = -L(2)*sin(T(1) + T(2));
    J21 = L(1)*cos(T(1)) + L(2)*cos(T(1) + T(2));
    J22 = L(2)*cos(T(1) + T(2));
    Jacobian = [[J11 J12]; [J21 J22]];

end

% Function to solve Inverse Kinematics Problem
function IK_solver = IK(L, des_pos)
    
    % Parameters
    Final_x = des_pos(1);
    Final_y = des_pos(2);
    L1 = L(1);
    L2 = L(2);
    
    % Elbow up or Elbow down 
    if(des_pos(1) >= 0)
        Elbow_up = (-1);
    else
        Elbow_up = 1;
    end

    % Inverse Kinematics Calculation Start

    % End Effector State (R, Theta)
    R = sqrt((Final_y*Final_y) + (Final_x*Final_x));
    Theta = atan2(Final_y, (Final_x + 0.00001));
    
    % Intermediate Angles
    Beta_raw = acos(((R*R)-(L1*L1)-(L2*L2))/(2*L1*L2));
    Beta = Beta_raw;
    Alpha = acos(((L1*L1) + (R*R) - (L2*L2))/(2*L1*R));
    
    % Final Joint Angles in Radians
    Final_Angle_1 = Theta + Alpha;
    Final_Angle_2 = Beta - pi;
    
    % Final Joint Angles in Degrees
    %Final_Angle_1 = Final_Angle_1*(180/pi);
    %Final_Angle_2 = Final_Angle_2*(180/pi);
    
    % Inverse Kinematics Calculation End
    IK_solver = ([Final_Angle_1 Final_Angle_2]);

end

% Function to solve Forward Kinematics Problem
function FK_Solver = FK(L, T)

    x_curr = L(1)*cos(T(1)) + L(2)*cos(T(1) + T(2));
    y_curr = L(1)*sin(T(1)) + L(2)*sin(T(1) + T(2));
    FK_Solver = [x_curr y_curr];

end



