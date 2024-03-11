% Matrix converting angular velocity to euler angle rates
function [T] = Angular2Euler(euler)
    alpha = euler(1);
    beta = euler(2);
    gamma = euler(3);

    T = [1, sin(alpha)*tan(beta), cos(alpha)*tan(beta); ...
        0, cos(alpha), -sin(gamma); ...
        0 sin(alpha)*sec(beta), cos(alpha)*sec(beta)];
end