clear

% lbr =  loadrobot('kukaIiwa14');
% 
% show(lbr);

% robot parameters
radius_B_left = 0.084; % [m]
radius_B_right = 0.084; % [m]

left_angle_shift_B = 6; %[deg]
right_angle_shift_B = 19; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;
P_offset = 6.5; %[deg]

om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
delta_base = [om1, pi/3 - om2, 2*pi/3+om1, pi - om2, 4*pi/3+om1, 5*pi/3 - om2];

om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = P_offset * pi/180.0 + [11 * pi/6 + om3, pi/2 - om4, pi / 2 + om3, 7*pi/6 - om4, 7*pi/6 + om3, 11*pi/6 - om4];


radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];
radius_P = repmat(0.053,6); %[m]

BASE_HEIGHT = 0.0665;
ACT_HEIGHT = 0.13045;
VCM_HEIGHT = 0.06395;
STAGE_HEIGHT = 0.0245;
LINK_HEIGHT = 0.0811;

%% Load Stewart Platform

assem = rigidBodyTree("DataFormat","row");

joint_body_arr = rigidBody.empty(0,6);
joint_arr = rigidBodyJoint.empty(0,6);
act_body_arr = rigidBody.empty(0,6);
act_arr = rigidBodyJoint.empty(0,6);
stage_body_arr = rigidBody.empty(0,6);
stage_arr = rigidBodyJoint.empty(0,6);

stewart_link_arr = rigidBodyTree.empty(0,6);
vcm_arr = rigidBodyTree.empty(0,6);

%% stage
% % Create 3 bodies for translation
% stage_body_x = rigidBody("stage_body_x");
% stage_body_y = rigidBody("stage_body_y");
% stage_body_z = rigidBody("stage_body_z");
% 
% % Create 3 bodies for rotation
% stage_body_a = rigidBody("stage_body_a");
% stage_body_b = rigidBody("stage_body_b");
% stage_body_c = rigidBody("stage_body_c");
% 
% % create 3 revolute joints for rotation A, B, C
% stage_rot_x = rigidBodyJoint("stage_rot_x",'prismatic');
% stage_rot_y = rigidBodyJoint("stage_rot_y",'prismatic');
% stage_rot_z = rigidBodyJoint("stage_rot_z",'prismatic');
% 
% stage_rot_a = rigidBodyJoint("stage_rot_a",'revolute');
% stage_rot_b = rigidBodyJoint("stage_rot_b",'revolute');
% stage_rot_c = rigidBodyJoint("stage_rot_c",'revolute');
% 
% % Set transform
% setFixedTransform(stage_rot_x,trvec2tform([0,0,BASE_HEIGHT+VCM_HEIGHT+LINK_HEIGHT+STAGE_HEIGHT/2]));
% setFixedTransform(stage_rot_y,trvec2tform([0,0,0]));
% setFixedTransform(stage_rot_z,trvec2tform([0,0,0]));
% setFixedTransform(stage_rot_a,trvec2tform([0,0,0]));
% setFixedTransform(stage_rot_b,trvec2tform([0,0,0]));
% setFixedTransform(stage_rot_c,trvec2tform([0,0,0]));
% 
% 
% % define translation direction X, Y, Z
% % define rotation direction A, B, C
% stage_rot_x.JointAxis = [1 0 0];
% stage_rot_y.JointAxis = [0 1 0];
% stage_rot_z.JointAxis = [0 0 1];
% stage_rot_a.JointAxis = [1 0 0];
% stage_rot_b.JointAxis = [0 1 0];
% stage_rot_c.JointAxis = [0 0 1];
% 
% % connect joints to refering bodies
% stage_body_x.Joint = stage_rot_x;
% stage_body_y.Joint = stage_rot_y;
% stage_body_z.Joint = stage_rot_z;
% stage_body_a.Joint = stage_rot_a;
% stage_body_b.Joint = stage_rot_b;
% stage_body_c.Joint = stage_rot_c;
% 
% % Add the collision bodies to the rigid body objects
% collStage = collisionCylinder(0.0665, STAGE_HEIGHT);
% addCollision(stage_body_x,collStage);
% 
% addBody(assem, stage_body_x,'base');
% addBody(assem, stage_body_y,'stage_body_x');
% addBody(assem, stage_body_z,'stage_body_y');
% addBody(assem, stage_body_a,'stage_body_z');
% addBody(assem, stage_body_b,'stage_body_a');
% addBody(assem, stage_body_c,'stage_body_b');

% stage_body = rigidBody("stage_body");
% jntStage = rigidBodyJoint("jntStage","fixed");
% setFixedTransform(jntStage,trvec2tform([0,0,BASE_HEIGHT+VCM_HEIGHT+LINK_HEIGHT+STAGE_HEIGHT/2]));
% 
% collStage = collisionCylinder(0.0665, STAGE_HEIGHT);
% addCollision(stage_body,collStage);
% 
% addBody(assem, stage_body,'base');



%% base
base_body = rigidBody("base_body");
jntBase = rigidBodyJoint("jntBase","fixed");
setFixedTransform(jntBase,trvec2tform([0,0, BASE_HEIGHT/2]));
base_body.Joint = jntBase;

collBase = collisionCylinder(0.0925, BASE_HEIGHT);
addCollision(base_body,collBase);
%addVisual(base_body, "cylinder", [0.0925, BASE_HEIGHT]);

addBody(assem, base_body,'base');

%% 6 linear actuators
for i = 1:6
    act_body_arr(i) = rigidBody(sprintf('act_body%d',i));
    act_arr(i) = rigidBodyJoint(sprintf('act%d',i),'prismatic');
    act_arr(i).JointAxis = [0,0,1];

    setFixedTransform(act_arr(i),trvec2tform([radius_B(i) * cos(delta_base(i)), radius_B(i) * sin(delta_base(i)), (ACT_HEIGHT)/2]));
    act_body_arr(i).Joint = act_arr(i);
    addBody(assem, act_body_arr(i),'base_body');

    vcm_arr(i) = Build_vcm_rbt(i);
    addSubtree(assem, act_body_arr(i).Name,vcm_arr(i));

end 

%% 6 links
for i = 1:6
    joint_body_arr(i) = rigidBody(sprintf('joint_body%d',i));
    joint_arr(i) = rigidBodyJoint(sprintf('joint%d',i),'revolute');
    setFixedTransform(joint_arr(i),trvec2tform([0,0, VCM_HEIGHT/2]));
    
    joint_body_arr(i).Joint = joint_arr(i);
    addBody(assem,joint_body_arr(i), sprintf('act_body%d',i));

    stewart_link_arr(i) = Build_link_rbt(i);
    addSubtree(assem,joint_body_arr(i).Name, stewart_link_arr(i));
    % add joint object to rigidBody
end 

%% Stage
stage_body = rigidBody("stage_body");
jntStage = rigidBodyJoint("jntStage","fixed");
setFixedTransform(jntStage,trvec2tform([-radius_P(1) * cos(delta_platform(1)), -radius_P(1) * sin(delta_platform(1)),STAGE_HEIGHT/2]));

stage_body.Joint = jntStage;

collStage = collisionCylinder(0.0665, STAGE_HEIGHT);
addCollision(stage_body,collStage);
addVisual(stage_body, "cylinder", [0.0665, STAGE_HEIGHT]);

addBody(assem, stage_body,'extruder_f_1');

%% Testing
% target_body = rigidBody("target_body");
% jntTarget = rigidBodyJoint("jntTarget","fixed");
% setFixedTransform(jntTarget,trvec2tform([-radius_P(3)*cos(delta_platform(3)),-radius_P(3) * sin(delta_platform(3)),0.05]));
% target_body.Joint = jntTarget;
% 
% addVisual(target_body, "sphere",0.005);
% addBody(assem, target_body,"stage_body");


%% Display results
show(assem,'Collisions','off',"frames","off"); view(0,90)
%show(assem,'Collisions','on')
gik1 = generalizedInverseKinematics("RigidBodyTree",assem,"ConstraintInputs",{"joint"});
gik1.SolverParameters.MaxIterations = 20;
gui = interactiveRigidBodyTree(assem,"MarkerScaleFactor",0.4, 'IKSolver',gik1,'Frames','off');

%% Constraints
% * eul2tform([-delta_platform(3),0,0])
% 1. Joint- Stage distance constraints
poseTgt1 = constraintPoseTarget('stage_body');
poseTgt1.ReferenceBody = 'extruder_f_2';
poseTgt1.TargetTransform = trvec2tform([-radius_P(2)*cos(delta_platform(2)),-radius_P(2) * sin(delta_platform(2)),0]);

poseTgt2 = constraintPoseTarget('stage_body');
poseTgt2.ReferenceBody = 'extruder_f_3';
poseTgt2.TargetTransform = trvec2tform([-radius_P(3)*cos(delta_platform(3)),-radius_P(3) * sin(delta_platform(3)),0]);

poseTgt3 = constraintPoseTarget('stage_body');
poseTgt3.ReferenceBody = 'extruder_f_4';
poseTgt3.TargetTransform = trvec2tform([-radius_P(4)*cos(delta_platform(4)),-radius_P(4) * sin(delta_platform(4)),0]);

poseTgt4 = constraintPoseTarget('stage_body');
poseTgt4.ReferenceBody = 'extruder_f_5';
poseTgt4.TargetTransform = trvec2tform([-radius_P(5)*cos(delta_platform(5)),-radius_P(5) * sin(delta_platform(5)),0]);

poseTgt5 = constraintPoseTarget('stage_body');
poseTgt5.ReferenceBody = 'extruder_f_6';
poseTgt5.TargetTransform = trvec2tform([-radius_P(6)*cos(delta_platform(6)),-radius_P(6) * sin(delta_platform(6)),0]);


% 2. VCM stroke constraint
strokeConstraint = constraintJointBounds(assem);
for i = 1:6
 strokeConstraint.Bounds(i,:) = [0, 0.03]; % max stroke 30mm
end

addConstraint(gui, poseTgt1);
addConstraint(gui, poseTgt2);
addConstraint(gui, poseTgt3);
addConstraint(gui, poseTgt4);
addConstraint(gui, poseTgt5);
addConstraint(gui, strokeConstraint);


% Plot target
target_body = rigidBody("target_body");
jntTarget = rigidBodyJoint("jntTarget","fixed");
addVisual(target_body, "Sphere", 0.005);

setFixedTransform(jntTarget,trvec2tform([0, 0, 0.3]));
target_body.Joint = jntTarget;

addBody(assem,target_body,"base");


%% Trajectory planning
gik1 = generalizedInverseKinematics("RigidBodyTree",assem,"ConstraintInputs",{"position","joint"});

strokeConstraint = constraintJointBounds(assem);
for i = 1:6
 strokeConstraint.Bounds(i,:) = [0, 0.03]; % max stroke 30mm
end

distanceFromTarget = constraintPositionTarget('target_body');
distanceFromTarget.ReferenceBody = 'stage_body';
distanceFromTarget.PositionTolerance = 0.000000001;


numWaypoints = 5;
intermediateDistance = 0.5;
finalDistanceFromTarget = 0.05;
q0 = homeConfiguration(assem);
qWaypoints=repmat(q0, numWaypoints,1);

distanceFromTargetValues =  linspace(intermediateDistance, finalDistanceFromTarget, numWaypoints-1);
for k= 2:numWaypoints
    distanceFromTarget.TargetPosition(3) = distanceFromTargetValues(k-1);
    [qWaypoints(k,:),solutionInfo] = gik1(qWaypoints(k-1,:), distanceFromTarget, strokeConstraint);
end
                                          
% Visualize generated trajectory
% Interpolate between the waypoints to generate a smooth trajectory. Use pchip to avoid overshoots,
% which might violate the joint limits of the robot.
framerate = 15;
r = rateControl(framerate);
tFinal = 10;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
numFrames = tFinal*framerate;
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';

stagePosition = zeros(numFrames,3);
for k = 1:numFrames
 stagePosition(k,:) = tform2trvec(getTransform(assem,qInterp(k,:), ...
 "stage_body"));
end 

%Show the robot in its initial configuration along with target
figure;
show(assem, qWaypoints(1,:), 'PreservePlot', false, "Frames","off");
hold on
p = plot3(stagePosition(1,1), stagePosition(1,2), stagePosition(1,3));

% Animate the manipulator
hold on
for k = 1:size(qInterp,1)
 show(assem, qInterp(k,:), 'PreservePlot', false,"Collisions","on","Frames","off");
 p.XData(k) = stagePosition(k,1);
 p.YData(k) = stagePosition(k,2);
 p.ZData(k) = stagePosition(k,3);
 waitfor(r);
end
hold off