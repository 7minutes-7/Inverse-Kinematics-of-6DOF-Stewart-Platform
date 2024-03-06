% command: edit startup to add search directories
%% Robot parameters
% joint_thickness = 10.5;
% 
% L = 80.6 + joint_thickness;
% radius_P =52;
% radius_B_left = 84;
% radius_B_right = 84;
% theta_max = 30; 
% joint_height = 17;
% platform_thickness = 6;
% link_radius = 3;
% base_joint_radius = 19.723083/2;
% stage_joint_radius = 19.723083/2;
% joint_rotation_P_left = 80;  % [deg]
% joint_rotation_P_right = -10;  % [deg]
% joint_rotation_B_left = 0; %[deg]
% joint_rotation_B_right = 0; %[deg]
% left_angle_shift_B = 6; %[deg]
% right_angle_shift_B = 19; %[deg]
% left_angle_shift_P = 20;
% right_angle_shift_P = 20;

% Before ver 5
L =80.6 + 10;
radius_P = 25;
radius_B_left = 65.8676832;
radius_B_right = 56.25;
theta_max = 30;
joint_height = 14;
platform_thickness = 6;
joint_thickness = 10;
base_joint_radius = 17.573379/2;
stage_joint_radius = 16.620867/2;
joint_rotation_P_left = 60;
joint_rotation_P_right = 60;
joint_rotation_B_left = 145.49430212;
joint_rotation_B_right = 39.17;
left_angle_shift_B = 6.5582618; %[deg]
right_angle_shift_B = 14.69; %[deg]
left_angle_shift_P = 0;
right_angle_shift_P = 0;

f = sym(zeros(6,1));
f_inv = sym(zeros(6,1));
theta = zeros(6,1);

%% Take input
% input initial position
x0 = input('Enter initial translation vector [x,y,z] or to exit, enter -1: ')';
if(x0 == -1)
   disp('Exiting program...')
   return;
end

angles0 = input('Enter initial angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

try
    theta0 =  calc_motor_displacement(x0,angles0(1),angles0(2),angles0(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
    % Output results
    disp('Motor displacement:')
    disp(theta0)
catch ME
    if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
       error(ME.message)
    end 
end

% input target position
x = input('Enter target translation vector [x,y,z] or to exit, enter -1: ')';
if(x == -1)
   disp('Exiting program...')
   return;
end

angles = input('Enter target angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

try
    theta =  calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
    % Output results
    disp('Motor displacement:')
    disp(theta)
catch ME
    if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
       error(ME.message)
    end 
end


%% Calculate numerical forward kinematics based on the Newton-Raphson method
xi0 = [x0(1), x0(2), x0(3), angles0(1), angles0(2), angles0(3)]'; % initial guess 
xi = xi0;

theta_target = calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);

allowed_err = 10^-5;

op = 0.6;
a = op;

i = 0;
while(true)
    xi_old = xi;
    
    theta_ik = calc_motor_displacement(xi(1:3), xi(4), xi(5), xi(6), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
    J_inverse = calc_jacobian_inverse(xi(1:3), xi(4), xi(5), xi(6), theta_ik);
    
    delta_xi =0.1*((J_inverse)\(theta_ik - theta_target));
    %delta_xi = (J_inverse)*
    xi = xi_old - delta_xi;
  
    err = norm(delta_xi);

    disp("Iteration")
    disp(i);
    disp(xi');
    disp(" ");

    i = i+1;
    if err < allowed_err
        break
    end 
end

disp(xi); % display calculated forward kinematics



 