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
radius_B = 45.75;
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

% 1. 6 motors distributed evenly
delta = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6]';

% 2. 2/2/2 triangular distribution
%om = asin(motor_radius/radius_B);
%delta = [pi/3 - om, pi/3 + om, pi - om, pi + om, 5*pi/3 - om, 5*pi/3 + om]';

%% calculate kinematics
R_PtoB = calc_RMatrix(alpha,beta,gamma);

% subtract offsets
x = x - [0,0, joint_height + motor_height + motor_thickness + base_thickness]' - R_PtoB*[0,0,platform_thickness + joint_height]';


ME1 = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
ME2 = MException('StewartPlatform:linkCollision', ...
                 'Link is colliding with the joint');

for i=1:6
    p = radius_P * [cos(delta(i)),sin(delta(i)),0]'; % platform coordinates
    mu = radius_B * [cos(delta(i)), sin(delta(i)), 0]'; % base coordinates

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
       theta(i) = t2; 
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
