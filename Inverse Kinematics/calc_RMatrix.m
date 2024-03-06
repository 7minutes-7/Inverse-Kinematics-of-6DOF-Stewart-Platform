%% inputs
% alpha: rotation about the z axis [rad]
% beta: rotation about y' axis [rad]
% gamma: rotation about x" axis [rad]

function[R] =calc_RMatrix(alpha,beta,gamma)
% calculate roatation matrix 

% ZYX (rotated in the order of XYZ) Euler angles
R = [cos(alpha) * cos(beta), sin(gamma)* sin(beta)* cos(alpha) - cos(gamma) * sin(alpha), cos(gamma)* sin(beta)* cos(alpha) + sin(gamma) * sin(alpha);
     cos(beta)* sin(alpha), sin(gamma)* sin(beta)* sin(alpha) + cos(gamma) * cos(alpha), cos(gamma)* sin(beta)* sin(alpha) - sin(gamma) * cos(alpha);
	-sin(beta), sin(gamma)* cos(beta), cos(gamma)* cos(beta)];

end