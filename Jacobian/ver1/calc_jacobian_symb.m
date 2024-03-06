syms x y z alpha beta gamma real; % add real to avoid conj()

radius_S = 25.0; %[mm]
radius_B_odd = 65.86; %[mm]
radius_B_even = 56.25; %[mm]
om1 = 6.5582618; %[deg] % left angle shift
om2 = 14.69;  %[deg]  % right angle shift
link_length = 90.6; %[mm]

f = sym(zeros(6,1));
f_inv = sym(zeros(6,1));
theta = zeros(6,1);

radius_B = [radius_B_odd, radius_B_even, radius_B_odd, radius_B_even, radius_B_odd, radius_B_even];

% Change units to [m], [rad]
% delta_stage = [pi/6, pi/2, 5*pi/6, 7*pi/6, 3*pi/2, 11*pi/6];
delta_stage = deg2rad([30, 90, 150, 210, 270, 330]);
delta_base = deg2rad([30-om1, 90+om2, 150-om1, 210+om2, 270 - om1, 330 +om2]);
radius_S = radius_S * 0.001;
radius_B = radius_B * 0.001;

for i = 1:6
    Sx = x + cos(alpha)*cos(beta)*radius_S*cos(delta_stage(i))+(cos(alpha)*sin(beta)*sin(gamma)-cos(gamma)*sin(alpha))*radius_S*sin(delta_stage(i))- radius_B(i)*cos(delta_base(i));
    Sy = y + cos(beta)*sin(alpha)*radius_S*cos(delta_stage(i)) + (cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))*radius_S*sin(delta_stage(i)) - radius_B(i)*sin(delta_base(i));
    Sz = z - sin(beta)*radius_S*cos(delta_stage(i)) + cos(beta)*sin(gamma)*radius_S*sin(delta_stage(i));
    f_inv(i) = Sz - sqrt(link_length*link_length - Sx^2 - Sy^2);
end

%vpa() gives a numerical approximation to the symbolic coefficients
% single (') is a conjugate transpose. Do (.') to avoid conjugates
jacobian_inverse_transpose = vpa(jacobian(f_inv,[x,y,z,alpha,beta,gamma]).', 6); 
