syms x y z alpha beta gamma real; % add real to avoid conj()

%% Robot parameters
joint_thickness = 10.5; 

L = 70.6 + joint_thickness;
radius_P =53;
radius_B_left = 84;
radius_B_right = 84;
joint_height = 17.5;
platform_thickness = 7.0;
left_angle_shift_B = 19; %[deg]
right_angle_shift_B = 6; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;
P_offset = 6.5; %[deg]

f = sym(zeros(6,1));
f_inv = sym(zeros(6,1));
theta = zeros(6,1);

%% Calculate remaining constants from given input
om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
%delta_base = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];

delta_base = [om1, pi/3 - om2, 2*pi/3+om1, pi - om2, 4*pi/3+om1, 5*pi/3 - om2];

om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = P_offset * pi/180.0 + [11 * pi/6 + om3, pi/2 - om4, pi / 2 + om3, 7*pi/6 - om4, 7*pi/6 + om3, 11*pi/6 - om4];


radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

% radius_P = radius_P * 0.001;
% radius_B = radius_B * 0.001;


% Take input
    trans = input('Enter translation vector [x,y,z] or to exit, enter -1: ')';
    
    if(trans == -1)
        disp('Exiting program...')
        return;
    end

    angles = input('Enter angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

    try
        calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
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

    R_PtoB = calc_RMatrix(alpha,beta,gamma);

    %p = radius_S * [cos(delta_stage(i)),sin(delta_stage(i)),0]'; % platform coordinates
    %q = [x,y,z]' + R_PtoB*p; % base coordinates
    %s = q - mu; % base coordinates

    for i = 1:6
        p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
        q = [x,y,z]' - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]' + R_PtoB*p; % base coordinates
        
        Sx = q(1) - radius_B(i)*cos(delta_base(i));
        Sy = q(2) - radius_B(i)*sin(delta_base(i));
        Sz = q(3);
        f_inv(i) = Sz - sqrt(L*L - Sx^2 - Sy^2);
    end
    
    %vpa() gives a numerical approximation to the symbolic coefficients
    % single (') is a conjugate transpose. Do (.') to avoid conjugates
    jacobian_inverse_transpose = vpa(jacobian(f_inv,[x,y,z,gamma,beta,alpha]).', 6); 

    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, x, trans(1)*0.001);
    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, y, trans(2)*0.001);
    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, z, trans(3)*0.001);
    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, alpha, angles(1)*pi/180.0);
    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, beta, angles(2)*pi/180.0);
    % jacobian_inverse_transpose = subs(jacobian_inverse_transpose, gamma, angles(3)*pi/180.0);

    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, x, trans(1));
    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, y, trans(2));
    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, z, trans(3));
    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, alpha, angles(1)*pi/180.0);
    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, beta, angles(2)*pi/180.0);
    jacobian_inverse_transpose = subs(jacobian_inverse_transpose, gamma, angles(3)*pi/180.0);
    disp(jacobian_inverse_transpose); % display output
