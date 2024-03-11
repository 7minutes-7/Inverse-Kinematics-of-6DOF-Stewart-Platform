% Converts given twist matrix representation (4x4) to twist vector (6x1)
function [V] = Matrix2Twist(M)
   w = [M(3,2); M(3,1); -M(2,1)];
   v = M(1:3,4);

   V = [w;v];
end
