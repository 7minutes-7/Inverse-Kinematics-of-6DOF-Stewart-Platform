% Define template for inverse matrix
syms M [6 6]
iM = inv(M);

% Define symbolic variables
syms theta1 theta2 theta3 theta4 theta5 theta6 phi1 phi2 phi3 phi4 phi5 phi6 real;

% Define manipulator characteristics
radius_S = 0.025; %[m]
radius_B_odd = 0.06586; %[m]
radius_B_even = 0.05625; %[m]
om1 = 6.5582618; %[deg] % left angle shift
om2 = 14.69;  %[deg]  % right angle shift
L = 0.0906; %[m] link length

radius_B = [radius_B_odd, radius_B_even, radius_B_odd, radius_B_even, radius_B_odd, radius_B_even];
delta_B = deg2rad([30-om1, 90+om2, 150-om1, 210+om2, 270 - om1, 330 +om2]);

% Link-stage connection point (x,y,z)'
ls = sym(zeros(6,3));
phi = [phi1,phi2,phi3,phi4,phi5,phi6];
theta = [theta1,theta2,theta3,theta4,theta5,theta6];

for i=1:6
    ls(i,:) = [radius_B(i)*cos(delta_B(i)) - L*cos(phi(i))*cos(delta_B(i)), radius_B(i)*sin(delta_B(i))-L*cos(phi(i))*sin(delta_B(i)), theta(i)+L*sin(phi(i))];
end


% Define 6 constraint equations for 6-DOF parallel manipulator
eta = sym(zeros(6,1));

for i=1:5
    eta(i)=sum((ls(i+1,:)-ls(i,:)).^2)-radius_S^2;
end
eta(6)=sum((ls(6,:)-ls(1,:)).^2)-radius_S^2;

% Define partial derivative jacobian of 6 constraint equations
K = jacobian(eta, theta);
K_star = jacobian(eta, phi);

% Define jacobian for end effector linear velocity
S_centroid = mean(ls,1)'; % 3*1 : coordinates of stage center

Jv = jacobian(S_centroid, theta);
Jv_star = jacobian(S_centroid, phi);

% Define jacobian for end effector angular velocity

% Define Jv equivalent [3*n]
K_star_inv = subs(iM, M, K_star);
Jv_eq = Jv - Jv_star*K_star_inv*K;
