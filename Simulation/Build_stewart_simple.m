% Stewart platform
% with base and stage for simplicity
function [assem] = Build_stewart_simple()
    %% Robot parameters
    BASE_HEIGHT = 0.008;
    LINK_HEIGHT = 0.2467;
    STAGE_HEIGHT = 0.007;


    %% stewart platform
    assem = rigidBodyTree("DataFormat","row");

    % Create base frame 
    stewart_base = rigidBody("stewart_base");
    jntBase = rigidBodyJoint("jntBase","fixed");
    stewart_base.Joint = jntBase;
    addVisual(stewart_base, "cylinder", [0.0925, BASE_HEIGHT]);
    
    
    % Create 3 bodies for translation
    stewart_stage_x = rigidBody("stewart_stage_x");
    stewart_stage_y = rigidBody("stewart_stage_y");
    stewart_stage_z = rigidBody("stewart_stage_z");
    
    % Create 3 bodies for rotation
    stewart_stage_a = rigidBody("stewart_stage_a");
    stewart_stage_b = rigidBody("stewart_stage_b");
    stewart_stage_c = rigidBody("stewart_stage_c");
    
    
    % create 3 revolute joints for rotation A, B, C
    stewart_rot_x = rigidBodyJoint("stewart_rot_x",'prismatic');
    stewart_rot_y = rigidBodyJoint("stewart_rot_y",'prismatic');
    stewart_rot_z = rigidBodyJoint("stewart_rot_z",'prismatic');
    
    stewart_rot_a = rigidBodyJoint("stewart_rot_a",'revolute');
    stewart_rot_b = rigidBodyJoint("stewart_rot_b",'revolute');
    stewart_rot_c = rigidBodyJoint("stewart_rot_c",'revolute');
    
    
    % Set transform
    setFixedTransform(stewart_rot_x,trvec2tform([0,0,LINK_HEIGHT]));
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
    stewart_stage_x.Joint = stewart_rot_x;
    stewart_stage_y.Joint = stewart_rot_y;
    stewart_stage_z.Joint = stewart_rot_z;
    stewart_stage_a.Joint = stewart_rot_a;
    stewart_stage_b.Joint = stewart_rot_b;
    stewart_stage_c.Joint = stewart_rot_c;
    
    % Add the visual to the rigid body objects
    addVisual(stewart_stage_c, "cylinder", [0.0665, STAGE_HEIGHT]);
    
    addBody(assem,stewart_base, "base");
    addBody(assem,stewart_stage_x, "stewart_base");
    addBody(assem,stewart_stage_y, "stewart_stage_x");
    addBody(assem,stewart_stage_z, "stewart_stage_y");
    addBody(assem,stewart_stage_a, "stewart_stage_z");
    addBody(assem,stewart_stage_b, "stewart_stage_a");
    addBody(assem,stewart_stage_c, "stewart_stage_b");
end 