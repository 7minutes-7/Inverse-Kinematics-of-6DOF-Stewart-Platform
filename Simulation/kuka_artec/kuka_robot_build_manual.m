% Kuka LBR Med 14 R820 
% robot model build manually
% https://kr.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html

close all
clear
clc

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0, 0, 0.36]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

robot = rigidBodyTree;

addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform = eul2tform([0, 0, -pi/2])*trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt2,tform);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = 0; % User defined
tform = eul2tform([0, 0, pi/2])*trvec2tform([0, 0, 0.42]); % User defined
setFixedTransform(jnt3,tform);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.HomePosition = 0; % User defined
tform = eul2tform([0, 0, pi/2])*trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt4,tform);
body4.Joint = jnt4;
addBody(robot,body4,'body3'); % Add body4 to body3

body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.HomePosition = 0; % User defined
tform = eul2tform([0, 0, -pi/2])*trvec2tform([0, 0, 0.4]); % User defined
setFixedTransform(jnt5,tform);
body5.Joint = jnt5;
addBody(robot,body5,'body4'); % Add body5 to body4

body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.HomePosition = 0; % User defined
tform = eul2tform([0, 0, -pi/2])*trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt6,tform);
body6.Joint = jnt6;
addBody(robot,body6,'body5'); % Add body6 to body5

body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');
jnt7.HomePosition = 0; % User defined
tform = eul2tform([0, 0, pi/2])*trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt7,tform);
body7.Joint = jnt7;
addBody(robot,body7,'body6'); % Add body7 to body6

bodyEndEffector = rigidBody('endeffector');
tform = trvec2tform([0, 0, 0.126]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform);
addBody(robot,bodyEndEffector,'body7');

showdetails(robot)

config = homeConfiguration(robot);
tform = getTransform(robot,config,'endeffector','base');

figure(Name="KUKA LBR MED 14 R820 Robot Model")
show(robot);