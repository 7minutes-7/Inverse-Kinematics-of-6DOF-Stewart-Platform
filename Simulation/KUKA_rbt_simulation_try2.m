clear
lbr =  loadrobot('kukaIiwa14');


%% stewart platform
% Create 3 bodies for translation
stewart_body_x = rigidBody("stewart_body_x");
stewart_body_y = rigidBody("stewart_body_y");
stewart_body_z = rigidBody("stewart_body_z");

% Create 3 bodies for rotation
stewart_body_a = rigidBody("stewart_body_a");
stewart_body_b = rigidBody("stewart_body_b");
stewart_body_c = rigidBody("stewart_body_c");


% create 3 revolute joints for rotation A, B, C
stewart_rot_x = rigidBodyJoint("stewart_rot_x",'prismatic');
stewart_rot_y = rigidBodyJoint("stewart_rot_y",'prismatic');
stewart_rot_z = rigidBodyJoint("stewart_rot_z",'prismatic');

stewart_rot_a = rigidBodyJoint("stewart_rot_a",'revolute');
stewart_rot_b = rigidBodyJoint("stewart_rot_b",'revolute');
stewart_rot_c = rigidBodyJoint("stewart_rot_c",'revolute');


% Set transform
setFixedTransform(stewart_rot_x,trvec2tform([0,0,0.2427/2]));
setFixedTransform(stewart_rot_y,trvec2tform([0,0,0]));
setFixedTransform(stewart_rot_z,trvec2tform([0,0,0]));
setFixedTransform(stewart_rot_a,trvec2tform([0,0,0]));
setFixedTransform(stewart_rot_b,trvec2tform([0,0,0]));
setFixedTransform(stewart_rot_c,trvec2tform([0,0,0]));


% define translation direction X, Y, Z
% define rotation direction A, B, C
stewart_rot_x.JointAxis = [1 0 0];
stewart_rot_y.JointAxis = [0 1 0];
stewart_rot_z.JointAxis = [0 0 1];
stewart_rot_a.JointAxis = [1 0 0];
stewart_rot_b.JointAxis = [0 1 0];
stewart_rot_c.JointAxis = [0 0 1];

% connect joints to refering bodies
stewart_body_x.Joint = stewart_rot_x;
stewart_body_y.Joint = stewart_rot_y;
stewart_body_z.Joint = stewart_rot_z;
stewart_body_a.Joint = stewart_rot_a;
stewart_body_b.Joint = stewart_rot_b;
stewart_body_c.Joint = stewart_rot_c;

% Add the visual to the rigid body objects
addVisual(stewart_body_c, "cylinder", [0.01, 0.2427]);

addBody(lbr,stewart_body_x, lbr.Bodies{end}.Name);
addBody(lbr,stewart_body_y, "stewart_body_x");
addBody(lbr,stewart_body_z, "stewart_body_y");
addBody(lbr,stewart_body_a, "stewart_body_z");
addBody(lbr,stewart_body_b, "stewart_body_a");
addBody(lbr,stewart_body_c, "stewart_body_b");

 
%show(lbr, "Visuals","off","Collisions","on");
gui = interactiveRigidBodyTree(lbr,"MarkerScaleFactor",0.25);


