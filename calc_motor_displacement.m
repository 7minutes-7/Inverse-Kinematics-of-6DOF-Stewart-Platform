function [theta] = calc_motor_displacement(x,alpha,beta,gamma)
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

%% FIXED VARIABLES
L =80.6 + 10;
radius_P = 25;
radius_B_left = 65.8676832;
radius_B_right = 56.25;
radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

theta_max = 30; 
motor_height = 72.3;
joint_height = 14;
platform_thickness = 6;
link_radius = 3;
joint_thickness = 10;
base_joint_radius = 17.573379/2;
stage_joint_radius = 16.620867/2;
joint_rotation_P = 60 * pi/180;

joint_rotation_B_left = 145.49430212 * pi/180;
joint_rotation_B_right = 39.17* pi/180;
joint_rotation_B = [joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right];

% 1. 6 motors distributed
delta_platform = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6]';

om1 = 6.5582618*pi / 180 ; % left angle shift
om2 = 14.69*pi/180; % right angle shift
delta_base_shifted = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];


%% calculate kinematics
R_PtoB = calc_RMatrix(alpha,beta,gamma);
% subtract offsets
x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
ME1 = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
ME2 = MException('StewartPlatform:linkCollision', ...
                 'Link is colliding with the joint');

for i=1:6
    p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
    mu = radius_B(i) * [cos(delta_base_shifted(i)), sin(delta_base_shifted(i)), 0]'; % base coordinates
 
    q = x + R_PtoB*p; % base coordinates
    s = q - mu; % base coordinates

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
    l = s - [0,0,theta(i)]';
    jb=[-mu(2) * cos(joint_rotation_B(i)) - mu(1)*sin(joint_rotation_B(i)); -mu(2) * sin(joint_rotation_B(i)) + mu(1)*cos(joint_rotation_B(i)) ;0]; 

    ep_base = asin(link_radius/sqrt(base_joint_radius*base_joint_radius + joint_thickness*joint_thickness/4)) + atan(joint_thickness/(2*base_joint_radius));

    phi_b1 = acos(dot(l,mu) / (L*radius_B(i)));
    phi_b2 = acos(dot(l,jb)/ (L*radius_B(i)));

    if(~(phi_b1>0 && phi_b1<pi && phi_b2>=ep_base && phi_b2<=pi-ep_base))
        throw(ME2)
    end

    % Check if the link placement collides with a platform joint
    jp = [-p(2) * cos(joint_rotation_P) - p(1)*sin(joint_rotation_P); -p(2) * sin(joint_rotation_P) + p(1)*cos(joint_rotation_P) ;0]; % vector parallel to the first shaft

    ep_stage = asin(link_radius/sqrt(stage_joint_radius*stage_joint_radius + joint_thickness*joint_thickness/4)) + atan(joint_thickness/(2*stage_joint_radius));
    
    phi_p1 = acos(dot(l, R_PtoB*p) / (L*radius_P));
    phi_p2 = acos(dot(l, R_PtoB*jp)/ (L*radius_P));

    if(~(phi_p1>0 && phi_p1<pi && phi_p2>=ep_stage && phi_p2<=pi-ep_stage))
        throw(ME2)
    end
    
end 
