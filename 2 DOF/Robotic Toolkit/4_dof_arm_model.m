% Basic 4 R Robotic Manipulator

% Create Rigid Body Elements
Basic_4R_arm = rigidBodyTree("DataFormat","column");
base = Basic_4R_arm.Base;

% Create a series of linkages as rigidBody objects. The robot consists of a 
% rotating base, 2 rectangular arms, and a gripper.
rotatingBase = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
gripper = rigidBody("gripper");

% Create collision objects for each rigid body with different shapes and 
% dimensions. When you create the collision objects, the coordinate frame 
% is centered in the middle of the object by default. Set the Pose property
% to move the frame to the bottom of each body along the z-axis. Model the 
% gripper as a sphere for simplicity.
collBase = collisionCylinder(0.05,0.04); % cylinder: radius,length
collBase.Pose = trvec2tform([0 0 0.04/2]);
coll1 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
coll1.Pose = trvec2tform([0 0 0.15/2]);
coll2 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
coll2.Pose = trvec2tform([0 0 0.15/2]);
collGripper = collisionSphere(0.025); % sphere: radius
collGripper.Pose = trvec2tform([0 -0.015 0.025/2]);

% Add the collision bodies to the rigid body objects.
addCollision(rotatingBase,collBase)
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(gripper,collGripper)


% Each rigid body is attached using a revolute joint. Create the rigidBodyJoint
% objects for each body. Specify the x-axis as the axis of rotation for the
% rectangular arm joints. Specify the y-axis for the gripper. The default axis is the z-axis.
jntBase = rigidBodyJoint("base_joint","revolute");
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jntGripper = rigidBodyJoint("gripper_joint","revolute");

jnt1.JointAxis = [1 0 0]; % x-axis
jnt2.JointAxis = [1 0 0];
jntGripper.JointAxis = [0 1 0]; % y-axis


% Set transformations of the joint attachment between bodies. 
% Each transformation is based on the dimensions of the previous rigid body 
% length (z-axis). 
% Offset the arm joints in the x-axis to avoid collisions during rotation.
setFixedTransform(jnt1,trvec2tform([0 0 0.04]))
setFixedTransform(jnt2,trvec2tform([0 0 0.15]))
setFixedTransform(jntGripper,trvec2tform([0 0 0.15]))

% Assemble Robot
bodies = {base,rotatingBase,arm1,arm2,gripper};
joints = {[],jntBase,jnt1,jnt2,jntGripper};

figure("Name","Assemble Robot","Visible","on")
for i = 2:length(bodies) % Skip base. Iterate through adding bodies and joints.
            bodies{i}.Joint = joints{i};
            addBody(Basic_4R_arm,bodies{i},bodies{i-1}.Name)
            show(Basic_4R_arm,"Collisions","on","Frames","off");
            drawnow;
end

% Interact with Robot Model
% figure("Name","Interactive GUI")
% gui = interactiveRigidBodyTree(Basic_2R_arm,"MarkerScaleFactor",0.25);

interactiveGUI = interactiveRigidBodyTree(Basic_4R_arm);

% Generate Random Joint Configurations
% for i = 1:4
%   subplot(2,2,i)
%   config = randomConfiguration(Basic_2R_arm);
%   show(Basic_2R_arm,config);
% end

% Interactive GUI
% interactiveGUI = interactiveRigidBodyTree(Basic_2R_arm);
% interactiveGUI.Configuration = randomConfiguration(Basic_2R_arm);

% Save Joint Configurations
% addConfiguration(interactiveGUI)
% disp(interactiveGUI.StoredConfigurations)
