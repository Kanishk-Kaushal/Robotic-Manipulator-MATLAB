% Inverse Kinematics on a 2R Planar Robotic Manipulator
Basic_2R_arm = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

% Link Lengths
L1 = 0.3;
L2 = 0.3;

% Add 'link1' body with 'joint1' joint.
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(Basic_2R_arm, body, 'base');

% Add 'link2' body with 'joint2' joint.
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(Basic_2R_arm, body, 'link1');

% Add 'tool' end effector with 'fix1' fixed joint.
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(Basic_2R_arm, body, 'link2');

% Show details of the robot to validate the input properties.
showdetails(Basic_2R_arm)


% Define a circle to be traced over the course of 10 seconds. 
% This circle is in the xy plane with a radius of 0.15.
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% Use an inverseKinematics object to find a solution of robotic configurations
% that achieve the given end-effector positions along the trajectory.
% Pre-allocate configuration solutions as a matrix qs.
q0 = homeConfiguration(Basic_2R_arm);
ndof = length(q0);
qs = zeros(count, ndof);

% Create the inverse kinematics solver.
% Because the xy Cartesian points are the only important factors of the end-effector pose for this workflow.
% specify a non-zero weight for the fourth and fifth elements of the weight vector.
% All other elements are set to zero.
ik = inverseKinematics('RigidBodyTree', Basic_2R_arm);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

% NOTE: Inverse Kinematics Solver Algorithm: 'BFGS Gradient Projection'

% Loop through the trajectory of points to trace the circle. 
% Call the ik object for each point to generate the joint configuration that achieves the end-effector position.
% Store the configurations to use later.
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

% Animate the Solution
figure
show(Basic_2R_arm,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])


framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(Basic_2R_arm,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end



