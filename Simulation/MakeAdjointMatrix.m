% Make 6x6 adjoint representation matrix of 4x4 transform matrix

function [Ad] = MakeAdjointMatrix(T)
    Ad = zeros(6,6);
    
    R = T(1:3,1:3);
    p = T(1:3,4);

    Ad(1:3, 1:3) = R;
    Ad(4:6, 1:3) = skew(p) * R;
    Ad(4:6, 4:6) = R;
end