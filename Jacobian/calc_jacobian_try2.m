%% Robot parameters
joint_thickness = 10.5; 

L = 70.6 + joint_thickness;
radius_P =53;
radius_B_left = 84;
radius_B_right = 84;
theta_max = 30; 
joint_height = 17.5;
platform_thickness = 7.0;
link_radius = 3;
base_joint_radius = 19.723083/2;
stage_joint_radius = 19.723083/2;
joint_rotation_P_left = 10;  % [deg]
joint_rotation_P_right = -10;  % [deg]
joint_rotation_B_left = 0; %[deg]
joint_rotation_B_right = 0; %[deg]
left_angle_shift_B = 6; %[deg]
right_angle_shift_B = 19; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;

f = sym(zeros(6,1));
f_inv = sym(zeros(6,1));
theta = zeros(6,1);

% delta_stage = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6];
delta_stage = deg2rad([30, 90, 150, 210, 270, 330]);
delta_base = deg2rad([30 - left_angle_shift_B, 90 + right_angle_shift_B, 150 - left_angle_shift_B, 210 + right_angle_shift_B, 270 - left_angle_shift_B, 330 + right_angle_shift_B]);

    % Take input
    x = input('Enter translation vector [x,y,z] or to exit, enter -1: ')';
    if(x == -1)
        disp('Exiting program...')
        return;
    end

    angles = input('Enter angles of rotation [alpha(z), beta(y), gamma(x)]: ')';
    alpha = angles(1);
    beta = angles(2);
    gamma = angles(3);

    try
        theta = calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
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
            disp(ME.message)

        end 
    end
    %% calculate kinematics
    R_PtoB = calc_RMatrix(angles(1),angles(2),angles(3));
    
    % Change units
    radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right] * 0.001;
    radius_P = radius_P * 0.001;
    theta = theta * 0.001;

    % subtract offsets
    x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
    x = x*0.001; % x [m]

    % Link-stage connection point (x,y,z)'
    ls = zeros(3,6);

    % J1 and J2 inverse
    % Final jacobian inverse is J1*J2
    J1 = zeros(6,6);
    J2 = zeros(6,6); 

    for i=1:6
        p = radius_P * [cos(delta_stage(i)),sin(delta_stage(i)),0]'; % platform coordinates
        mu = radius_B(i) * [cos(delta_base(i)), sin(delta_base(i)), 0]'; % base coordinates
     
        q = x + R_PtoB*p; % base coordinates
        s = q - mu; % base coordinates
    
        ls(:,i) = s - [0,0,theta(i)]'; % 3x1
        
        J1(i,:) = cat(2, ls(:,i)', cross(R_PtoB*p ,ls(:,i))') / ls(3,i); % concatentate row
    end 

    % calculate J2 for euler angle rate->angular velocity
    H = [1,0,-sin(beta); 
        0, cos(alpha), sin(alpha)*cos(beta);
        0, -sin(alpha), cos(alpha)*cos(beta)];

    J2 = cat(2, eye(3,3), zeros(3,3));
    temp = cat(2, zeros(3,3),H);
    J2 = cat(1,J2, temp);
    %J2 = eye(6,6)
    J = J1*J2;

    disp(J') % display transpose of inverse
