function[R] =calc_RMatrix(alpha,beta,gamma)
% change to radians
alpha = alpha*pi/180;
beta = beta*pi/180;
gamma = gamma*pi/180;

% calculate roatation matrix 

% ZYX (rotated in the order of XYZ) Euler angles
R = [cos(beta) * cos(gamma), sin(alpha)* sin(beta)* cos(gamma) - cos(alpha) * sin(gamma), cos(alpha)* sin(beta)* cos(gamma) + sin(alpha) * sin(gamma);
     cos(beta)* sin(gamma), sin(alpha)* sin(beta)* sin(gamma) + cos(alpha) * cos(gamma), cos(alpha)* sin(beta)* sin(gamma) - sin(alpha) * cos(gamma);
	-sin(beta), sin(alpha)* cos(beta), cos(alpha)* cos(beta)];

end