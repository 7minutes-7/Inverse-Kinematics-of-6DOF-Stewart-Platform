% Kuka LBR Med 14 R820 
% robot model build with modified DH parameters
% https://kr.mathworks.com/help/robotics/ug/build-manipulator-robot-using-kinematic-dh-parameters.html

function [robot] = Func_kuka_robot_build_dhparam(dhparams)
    
    arguments
        dhparams = [
            0 0 0.36 0;
            0 -pi/2 0 0;
            0  pi/2 0.42 0;
            0 pi/2 0 0;
            0 -pi/2 0.4 0;
            0 -pi/2 0 0;
            0 pi/2 0 0
            ];
    end 
    
    ndof = 7;
    
    robot = rigidBodyTree;
    
    bodies = cell(ndof,1);
    joints = cell(ndof,1);
    for i = 1:ndof
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        setFixedTransform(joints{i},dhparams(i,:),"mdh");
        bodies{i}.Joint = joints{i};
        if i == 1 % Add first body to base
            addBody(robot,bodies{i},"base")
        else % Add current body to previous body by name
            addBody(robot,bodies{i},bodies{i-1}.Name)
        end
    end
    
    bodyEndEffector = rigidBody('endeffector');
    tform = trvec2tform([0, 0, 0.126]); % User defined
    setFixedTransform(bodyEndEffector.Joint,tform);
    addBody(robot,bodyEndEffector,'body7');
    
    % showdetails(robot)
    
    config = homeConfiguration(robot);
    tform = getTransform(robot,config,'endeffector','base');
    
    % figure(Name="KUKA LBR MED 14 R820 Robot Model")
    % show(robot);
end 