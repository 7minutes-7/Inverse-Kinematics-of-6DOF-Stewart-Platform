% Converts given twist matrix representation (4x4) to twist vector (6x1)
function [M] = Twist2Matrix(V)
   w = V(1:3);
   v = V(4:6);

   M = zeros(4,4);

   M(1:3, 1:3) = skew(w);
   M(1:3, 4) = v;
end
