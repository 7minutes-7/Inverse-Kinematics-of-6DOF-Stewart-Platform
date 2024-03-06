%% Robot parameters
joint_thickness = 10.5;

L = 80.6 + joint_thickness;
radius_P =52;
radius_B_left = 84;
radius_B_right = 84;
theta_max = 30; 
joint_height = 17;
platform_thickness = 6;
link_radius = 3;
base_joint_radius = 19.723083/2;
stage_joint_radius = 19.723083/2;
joint_rotation_P_left = 80;  % [deg]
joint_rotation_P_right = -10;  % [deg]
joint_rotation_B_left = 0; %[deg]
joint_rotation_B_right = 0; %[deg]
left_angle_shift_B = 6; %[deg]
right_angle_shift_B = 19; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;

%% Calculate kinematics

x = [0, 0, 123.3]';
angles = [0, 0, 0]';

try
theta = calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
catch ME
    if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            disp("impossible");
            return
    end 
end
display(theta)