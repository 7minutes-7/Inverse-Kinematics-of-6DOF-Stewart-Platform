function[R] =calc_RMatrix(alpha,beta,gamma)
% change to radians
alpha = alpha*pi/180;
beta = beta*pi/180;
gamma = gamma*pi/180;

% calculate roatation matrix
R = [cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)- sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
          sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
          -sin(beta), cos(beta)*sin(gamma), cos(beta)*cos(gamma)];
end