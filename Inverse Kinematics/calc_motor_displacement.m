%% Input
% x [mm]
% alpha, beta, gamma [rad]
function [theta] = calc_motor_displacement(x,alpha,beta,gamma, stewart_params)
%% Fixed // units: [rad]
% delta: angle to each point of hexagon 
% R_MutoB: rotation from mu to base (z-axis rotation)
% p: vector position of each link attached to platform
% mu: vector position of each link attached to base

%% Other constants in model [mm]
% L: length of link
% radius_P: radius of platform
% radius_B: radius of base
% theta_max: maximum displacement of motor
% motor_height
% motor_thickness
% joint_height
% platform_thickness
% base_thickness

%% validate inputs
arguments
        x double 
        alpha double 
        beta double
        gamma double

        stewart_params.L = 70.6 + 10.5; % default values
        stewart_params.radius_P = 53;
        stewart_params.radius_B_left = 84;
        stewart_params.radius_B_right = 84;
        stewart_params.theta_max = 30;
        stewart_params.joint_height = 17.5;
        stewart_params.platform_thickness = 7;
        stewart_params.link_radius = 3;
        stewart_params.joint_thickness = 10.5;
        stewart_params.base_joint_radius =19.723083/2;
        stewart_params.stage_joint_radius =19.723083/2;
        stewart_params.joint_rotation_P_left = 10;  % [deg]
        stewart_params.joint_rotation_P_right = -10;  % [deg]
        stewart_params.joint_rotation_B_left = 0; %[deg]
        stewart_params.joint_rotation_B_right = 0; %[deg]
        stewart_params.left_angle_shift_B = 19; %[deg]
        stewart_params.right_angle_shift_B = 6; %[deg]
        stewart_params.left_angle_shift_P = 20; %[deg]
        stewart_params.right_angle_shift_P = 20; %[deg]

        stewart_params.P_offset = 6.5; %[deg]

end

L = stewart_params.L;
radius_P = stewart_params.radius_P ;
radius_B_left = stewart_params.radius_B_left;
radius_B_right = stewart_params.radius_B_right;
theta_max = stewart_params.theta_max ;
joint_height = stewart_params.joint_height ;
platform_thickness = stewart_params.platform_thickness ;
link_radius = stewart_params.link_radius;
joint_thickness = stewart_params.joint_thickness ;
base_joint_radius = stewart_params.base_joint_radius ;
stage_joint_radius = stewart_params.stage_joint_radius ;
joint_rotation_P_left  = stewart_params.joint_rotation_P_left ;
joint_rotation_P_right  = stewart_params.joint_rotation_P_right ;
joint_rotation_B_left = stewart_params.joint_rotation_B_left;
joint_rotation_B_right = stewart_params.joint_rotation_B_right;
left_angle_shift_B = stewart_params.left_angle_shift_B;
right_angle_shift_B = stewart_params.right_angle_shift_B;
left_angle_shift_P = stewart_params.left_angle_shift_P;
right_angle_shift_P = stewart_params.right_angle_shift_P;
P_offset = stewart_params.P_offset;
%% Calculate remaining constants from given input
joint_rotation_P_left = joint_rotation_P_left* pi/180; %[rad]
joint_rotation_P_right = joint_rotation_P_right* pi/180; %[rad]
joint_rotation_B_left = joint_rotation_B_left * pi/180; %[rad]
joint_rotation_B_right = joint_rotation_B_right * pi/180; %[rad]
joint_rotation_B = [joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right];
joint_rotation_P = [joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right];

om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
%delta_base = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];

delta_base = [om1, pi/3 - om2, 2*pi/3+om1, pi - om2, 4*pi/3+om1, 5*pi/3 - om2];

om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = P_offset * pi/180.0 + [11 * pi/6 + om3, pi/2 - om4, pi / 2 + om3, 7*pi/6 - om4, 7*pi/6 + om3, 11*pi/6 - om4];


radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

%% calculate kinematics
theta = zeros(6,1);
R_PtoB = calc_RMatrix(alpha, beta, gamma);

% Reshape input to column matrix
x = reshape(x,[3,1]);

% subtract offsets
x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
ME1 = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
ME2 = MException('StewartPlatform:linkCollision', ...
                 'Link is colliding with the joint');
q_arr = zeros(3,6);
for i=1:6
    R_MuToB = calc_RMatrix(joint_rotation_B(i) + delta_base(i),0,0); % joint coordinates to base

    p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
    mu = radius_B(i) * [cos(delta_base(i)), sin(delta_base(i)), 0]'; % base coordinates
 
    q = x + R_PtoB*p; % base coordinates
    q_arr(:,i) = q; % for debugging purposes

    s = R_MuToB\(q - mu); % joint coordinates
    %s = (q - mu); % base coordinates

    r = L*L - s(1)*s(1) - s(2)*s(2);
    t1 = s(3) + sqrt(r);
    t2 = s(3) - sqrt(r); 

    if(r<0)
       throw(ME1)
    elseif(t1>=0 && t1<=theta_max)
       theta(i) = t1;
    elseif(t2>=0 && t2<=theta_max)
       theta(i) = t2; % Always chosen
    else 
       throw(ME1)
    end
    
    % Check if the link placement collides with a base joint
    l = s - [0,0,theta(i)]'; % base joint coordinates
    jb = [0,1,0]'; % base joint coordinates
    % jb=[-mu(2) * cos(joint_rotation_B(i)) - mu(1)*sin(joint_rotation_B(i)); -mu(2) * sin(joint_rotation_B(i)) + mu(1)*cos(joint_rotation_B(i)) ;0]; 

    ep_base = asin(link_radius/sqrt(base_joint_radius*base_joint_radius + joint_thickness*joint_thickness/4)) + atan(joint_thickness/(2*base_joint_radius));

    phi_b1 = acos(dot(R_MuToB*l,mu) / (L*radius_B(i)));
    phi_b2 = acos(dot(l,jb)/ (L*norm(jb)));

    if(~(phi_b1>0 && phi_b1<pi && phi_b2>=ep_base && phi_b2<=pi-ep_base))
        throw(ME2)
    end

    % Check if the link placement collides with a platform joint
    R_MuStoP = calc_RMatrix(joint_rotation_P(i) + delta_platform(i),0,0); % Platform -> platform joint coordinates

    jp =[0,1,0]'; % platform joint coordinates
    lpj = (R_PtoB \ (R_MuToB * l)); % platform coordinates

    %jp = [-p(2) * cos(joint_rotation_P(i)) - p(1)*sin(joint_rotation_P(i)); -p(2) * sin(joint_rotation_P(i)) + p(1)*cos(joint_rotation_P(i)) ;0]; % vector parallel to the first shaft
    
    ep_stage = asin(link_radius/sqrt(stage_joint_radius*stage_joint_radius + joint_thickness*joint_thickness/4)) + atan(joint_thickness/(2*stage_joint_radius));
    
    phi_p1 = acos(dot(lpj, p) / (L*radius_P));
    phi_p2 = acos(dot(R_MuStoP\lpj, jp)/ (L*norm(jp)));

    if(~(phi_p1>0 && phi_p1<pi && phi_p2>=ep_stage && phi_p2<=pi-ep_stage))
        throw(ME2)
    end
    
end 
