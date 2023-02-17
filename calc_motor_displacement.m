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
L = 90;
radius_P = 38;
radius_B = 51.5;
theta_max = 31.8;
motor_height = 50.8;
motor_thickness = 6.4;
motor_radius = 22.25;
joint_height = 18;
platform_thickness = 6;
base_thickness = 8;
link_radius = 3.5;
joint_thickness = 8;
joint_radius = 10;
joint_shift = 15.5;

% calculate radius after shifting
radius_B_shifted = sqrt((joint_shift+radius_B/2)^2 + (radius_B*sqrt(3)/2)^2);

% 1. 6 motors distributed evenly
delta_base = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6]';
delta_platform = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6]';
om = acos((radius_B_shifted^2 + radius_B^2-joint_shift^2)/(2*radius_B_shifted*radius_B));
delta_base_shifted = [pi/6 - om, pi/2 + om, 5*pi/6-om, 7*pi/6+om, 3*pi/2-om, 11*pi/6+om];

%% calculate kinematics
R_PtoB = calc_RMatrix(alpha,beta,gamma);
%Rz = [cos(-pi/6), -sin(-pi/6), 0; sin(-pi/6), cos(-pi/6), 0; 0, 0, 1];
% subtract offsets
x = x - [0,0, joint_height + motor_height + motor_thickness + base_thickness]' - R_PtoB*[0,0,platform_thickness + joint_height]';


ME1 = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
ME2 = MException('StewartPlatform:linkCollision', ...
                 'Link is colliding with the joint');

for i=1:6
    p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
    %mu = radius_B * [cos(delta_base(i)), sin(delta_base(i)), 0]'; % base coordinates

    % New coordinates after shifting
    mu = radius_B_shifted * [cos(delta_base_shifted(i)), sin(delta_base_shifted(i)), 0]'; 

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
    jb = [-mu(2), mu(1), 0]';

    ep = asin(link_radius/sqrt(joint_radius*joint_radius + joint_thickness*joint_thickness/4)) + atan(joint_thickness/(2*joint_radius));

    phi_b1 = acos(dot(l,mu) / (L*radius_B));
    phi_b2 = acos(dot(l,jb)/ (L*radius_B));

    if(~(phi_b1>0 && phi_b1<pi && phi_b2>=ep && phi_b2<=pi-ep))
        throw(ME2)
    end

    % Check if the link placement collides with a platform joint
    jp = [-p(2), p(1), 0]';
    
    phi_p1 = acos(dot(l, R_PtoB*p) / (L*radius_P));
    phi_p2 = acos(dot(l, R_PtoB*jp)/ (L*radius_P));

    if(~(phi_p1>0 && phi_p1<pi && phi_p2>=ep && phi_p2<=pi-ep))
        throw(ME2)
    end

end 
