clear
close all

%% Building robot

% Load KUKA robot
robot =  Func_kuka_robot_build_dhparam();

% Load stewart platform 
assem = Build_stewart_simple();

% Add stewart to KUKA end effector
addSubtree(robot, robot.Bodies{end}.Name, assem);

%showdetails(robot);
%% Moving robot
% set joint parameters
jointParams = zeros(1,13);
config = homeConfiguration(robot);

for i = 1:13
    config(i).JointPosition = jointParams(i);
end


%% Display results

% show(assem);
% gui = interactiveRigidBodyTree(assem,"MarkerScaleFactor",0.25);


% Show the robot in its initial configuration along with target
figure;
show(robot, config, 'PreservePlot', false, "Visuals","off","Frames","on");
xlim([-1,1.5]);
ylim([-1,1.5]);
zlim([-0.5, 2.0]);
hold on

jointValues = linspace(0,pi/2,50);
r = rateControl(5);

for n = 1: numel(jointValues)
    jointParams(2) = jointValues(n);
    config(2).JointPosition = jointParams(2);
   
    show(robot, config, 'PreservePlot', false, "Visuals","off","Frames","on");
    waitfor(r);
end 
hold off;
    


