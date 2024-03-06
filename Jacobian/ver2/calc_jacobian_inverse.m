%% Inputs
% x : stage translation of the x-axis
% y : stage translation of the y-axis
% z : stage translation of the z-axis
% alpha : stage euler angle of the z-axis
% beta : stage euler angle of the y'-axis
% gamma : stage euler angle of the x"-axis
% theta : calculated motor displacment through inverse kinematics

%% Ouputs
% J: inverse jacobian of the given stage configuration

function[J] = calc_jacobian_inverse(x,alpha,beta,gamma,theta)

% FIXED VARIABLES
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


%% Calculate remaining constants from given input
om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
%delta_base = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];

delta_base = [om1, pi/3 - om2, 2*pi/3+om1, pi - om2, 4*pi/3+om1, 5*pi/3 - om2];

om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = P_offset * pi/180.0 + [11 * pi/6 + om3, pi/2 - om4, pi / 2 + om3, 7*pi/6 - om4, 7*pi/6 + om3, 11*pi/6 - om4];


radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

    %% calculate kinematics
    R_PtoB = calc_RMatrix(alpha,beta,gamma);
    % subtract offsets
    x = reshape(x,[3,1]);
    theta = reshape(theta,[6,1]);
    
    x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
    

    % Link-stage connection point (x,y,z)'
    ls = zeros(3,6);
    ls_norm = zeros(3,6);

    % J1 and J2 inverse
    % Final jacobian inverse is J1*J2
    J1 = zeros(6,6);
    J2 = zeros(6,6); 

    for i=1:6
        p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
        mu = radius_B(i) * [cos(delta_base(i)), sin(delta_base(i)), 0]'; % base coordinates
     
        q = x + R_PtoB*p; % base coordinates
        s = q - mu; % base coordinates
    
        ls(:,i) = s - [0,0,theta(i)]'; % 3x1

        %ls_norm(:,i) = ls(:,i) / norm(ls(:,i));
        
        J1(i,:) = cat(2, ls(:,i)', cross(R_PtoB*p ,ls(:,i))') / ls(3,i); % concatentate row

        %J1(i,:) = cat(2, ls_norm(:,i)', cross(R_PtoB*p ,ls_norm(:,i))') * ls(3,i); % concatentate row
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

    %disp(J') % display transpose of inverse
end
