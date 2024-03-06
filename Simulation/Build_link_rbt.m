function [stewart_link] = Build_link_rbt(i)

stewart_link = rigidBodyTree("DataFormat","row");

% create 3 bodies for rotation A, B, C
extruder_a = rigidBody(sprintf('extruder_a_%d',i));
extruder_b = rigidBody(sprintf('extruder_b_%d',i));
extruder_c = rigidBody(sprintf('extruder_c_%d',i));

% create 3 bodies for rotation D, E, F
extruder_d = rigidBody(sprintf('extruder_d_%d',i));
extruder_e = rigidBody(sprintf('extruder_e_%d',i));
extruder_f = rigidBody(sprintf('extruder_f_%d',i));

link = rigidBody(sprintf('link_%d',i));
jntLink = rigidBodyJoint(sprintf('jntLink_%d',i));

% create 3 revolute joints for rotation A, B, C
rot_a = rigidBodyJoint(sprintf('rot_a_%d',i),'revolute');
rot_b = rigidBodyJoint(sprintf('rot_b_%d',i),'revolute');
rot_c = rigidBodyJoint(sprintf('rot_c_%d',i),'revolute');

% create 3 revolute joints for rotation D, E, F
rot_d = rigidBodyJoint(sprintf('rot_d_%d',i),'revolute');
rot_e = rigidBodyJoint(sprintf('rot_e_%d',i),'revolute');
rot_f = rigidBodyJoint(sprintf('rot_f_%d',i),'revolute');


setFixedTransform(rot_a,trvec2tform([0 0 0]))
setFixedTransform(rot_b,trvec2tform([0 0 0]))
setFixedTransform(rot_c,trvec2tform([0 0 0]))
setFixedTransform(jntLink,trvec2tform([0,0,0.0811/2]))% Link length 81.1mm
setFixedTransform(rot_d,trvec2tform([0 0 0.0811/2])) 
setFixedTransform(rot_e,trvec2tform([0 0 0]))
setFixedTransform(rot_f,trvec2tform([0 0 0]))


% define translation direction X, Y, Z
% define rotation direction A, B, C
rot_a.JointAxis = [1 0 0];
rot_b.JointAxis = [0 1 0];
rot_c.JointAxis = [0 0 1];


% define translation direction X, Y, Z
% define rotation direction D, E, F
rot_d.JointAxis = [1 0 0];
rot_e.JointAxis = [0 1 0];
rot_f.JointAxis = [0 0 1];

% connect joints to refering bodies
extruder_a.Joint = rot_a;
extruder_b.Joint = rot_b;
extruder_c.Joint = rot_c;
extruder_d.Joint = rot_d;
extruder_e.Joint = rot_e;
extruder_f.Joint = rot_f;
link.Joint = jntLink;

% Add the collision bodies to the rigid body objects
collLink = collisionCylinder(0.003, 0.0811);
addCollision(link,collLink);
addVisual(link, "cylinder", [0.003, 0.0811]);


% connect bodies to rigid 
addBody(stewart_link,extruder_a,'base')
addBody(stewart_link,extruder_b,sprintf('extruder_a_%d',i))
addBody(stewart_link,extruder_c,sprintf('extruder_b_%d',i))
addBody(stewart_link,link,sprintf('extruder_c_%d',i))
addBody(stewart_link,extruder_d,sprintf('link_%d',i))
addBody(stewart_link,extruder_e,sprintf('extruder_d_%d',i))
addBody(stewart_link,extruder_f,sprintf('extruder_e_%d',i))

%show(stewart_link,[0 0 0 0 0 pi],'Collisions','on','frames','off')

%figure("Name","Interactive GUI")
%gui = interactiveRigidBodyTree(stewart_test,"MarkerScaleFactor",0.25);

%save("link_rbt.mat","stewart_link");

end
