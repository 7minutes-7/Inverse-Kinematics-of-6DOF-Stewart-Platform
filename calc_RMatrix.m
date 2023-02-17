function[R] =calc_RMatrix(alpha,beta,gamma)
% change to radians
alpha = alpha*pi/180;
beta = beta*pi/180;
gamma = gamma*pi/180;

% calculate roatation matrix 

% ZYX (rotated in the order of XYZ)
R = [cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-cos(gamma)*sin(alpha), sin(alpha)*sin(gamma)+cos(alpha)*cos(gamma)*sin(beta);
    cos(beta)*sin(alpha), cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma), cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma);
    -sin(beta), cos(beta)*sin(gamma), cos(beta)*cos(gamma)];

end