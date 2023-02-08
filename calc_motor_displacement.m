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
delta = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6]';
L = 88;
radius_P = 38;
radius_B = 45;
theta_max = 15;
motor_height = 50.8;
motor_thickness = 6.4;
joint_height = 18;
platform_thickness = 6;
base_thickness = 8;

%% calculate kinematics
R_PtoB = calc_RMatrix(alpha,beta,gamma);

ME = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
  
for i=1:6
    p = radius_P*[cos(delta(i)), sin(delta(i)), 0]';
    mu = radius_B*[cos(delta(i)), sin(delta(i)), 0]';
    % z-axis rotation of delta
    R_MutoB = [cos(delta(i)), -sin(delta(i)), 0;... 
               sin(delta(i)), cos(delta(i)), 0;...
               0,0,1];
    q = x + R_PtoB*p;
    s = inv(R_MutoB)*(q-mu);  

    r = L*L - s(1)*s(1) - s(2)*s(2);
    t1 = s(3) + sqrt(r);
    t2 = s(3) - sqrt(r); 
    
    if(r<0)
       throw(ME)
    elseif(t1>=0 && t1<=theta_max)
       theta(i) = t1;
    elseif(t2>=0 && t2<=theta_max)
       theta(i) = t2; 
    else 
       throw(ME)
    end
end 