%% Calculating jacobian without partial derivatives

%% Robot parameters
joint_thickness = 10.5;

L = 80.6 + joint_thickness;
radius_P =52;
radius_B_left = 84;
radius_B_right = 84;
theta_max = 30; 
joint_height = 17;
platform_thickness = 6;
link_radius = 3;
base_joint_radius = 19.723083/2;
stage_joint_radius = 19.723083/2;
joint_rotation_P_left = 80;  % [deg]
joint_rotation_P_right = -10;  % [deg]
joint_rotation_B_left = 0; %[deg]
joint_rotation_B_right = 0; %[deg]
left_angle_shift_B = 6; %[deg]
right_angle_shift_B = 19; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;

%% Take input
% calculate initial theta based on given input transformation 
x0 = input('Enter initial translation vector [x,y,z] or to exit, enter -1: ')';
if(x0 == -1)
   disp('Exiting program...')
   return;
end

angles0 = input('Enter initial angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

try
    theta0 =  calc_motor_displacement(x0,angles0(1),angles0(2),angles0(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
    % Output results
    disp('Motor displacement:')
    disp(theta0)
catch ME
    if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
       error(ME.message)
    end 
end

% input target position
% calculate target theta based on given input transformation 
x = input('Enter target translation vector [x,y,z] or to exit, enter -1: ')';
if(x == -1)
   disp('Exiting program...')
   return;
end

angles = input('Enter target angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

try
    theta =  calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
    % Output results
    disp('Motor displacement:')
    disp(theta)
catch ME
    if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
       error(ME.message)
    end 
end

%% Calculate remaining constants from given input
joint_rotation_P_left = joint_rotation_P_left* pi/180; %[rad]
joint_rotation_P_right = joint_rotation_P_right* pi/180; %[rad]
joint_rotation_B_left = joint_rotation_B_left * pi/180; %[rad]
joint_rotation_B_right = joint_rotation_B_right * pi/180; %[rad]
joint_rotation_B = [joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right];
joint_rotation_P = [joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right];

om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
delta_base_shifted = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];


om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = [pi/3 - om3, pi/3 + om4, pi - om3, pi + om4, 5*pi/3 - om3, 5*pi/3 + om4];

radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

%% Matrices for newton-raphson
allowed_err = 10^-5;

D = zeros(6,6);
T = zeros(6,6);
J = zeros(6,6);
Jg = zeros(6,6);
g = zeros(6,1);
x_old = zeros(6,1);
x_new = zeros(6,1);

%% calculate kinematics
ME1 = MException('StewartPlatform:notPossible', ...
                 'This configuration is impossible');
% initial values
x_old = [x0(1),x0(2),x0(3),angles0(1),angles0(2),angles0(3)].'; 

% start iteration loop
while (true) 
    R_PtoB0 = calc_RMatrix(x_old(4),x_old(5),x_old(6));
    R_PtoB = calc_RMatrix(angles(1),angles(2),angles(3));
    
    % subtract offsets
    x0 = x_old(1:3);
    x0 = x0 - [0,0, joint_height]' - R_PtoB0*[0,0,platform_thickness + joint_height]';
    x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
    
    for i=1:6
        p = radius_P * [cos(delta_platform(i)),sin(delta_platform(i)),0]'; % platform coordinates
        mu = radius_B(i) * [cos(delta_base_shifted(i)), sin(delta_base_shifted(i)), 0]'; % base coordinates
        
        b0 = R_PtoB0*p;
        q0 = x0 + b0; % base coordinates
        s0 = q0 - mu; % base coordinates
        
        % check if transformation calculated by iteration is a possible one
        r = L*L - s0(1)*s0(1) - s0(2)*s0(2);
        t1 = s0(3) + sqrt(r);
        t2 = s0(3) - sqrt(r); 
        if(r<0)
           throw(ME1)
        elseif(t1>=0 && t1<=theta_max)
           theta0(i) = t1;
        elseif(t2>=0 && t2<=theta_max)
           theta0(i) = t2; % Always chosen
        else 
           throw(ME1)
        end
        
        q = x + R_PtoB*p; % base coordinates
        s = q - mu; % base coordinates

        % calculate J
        J(i,:) = cat(2, s0.', cross(b0,s0).');
    
       
        d_old = sqrt(L*L + 2*theta0(i)*s0(3)-theta0(i)*theta0(i));
        d_new = sqrt(L*L + 2*theta(i)*s(3)-theta(i)*theta(i));
    
        D(i,i) = d_new + d_old;
    
        % calculate g
        g(i) = d_new*d_new - dot(s0,s0);
        
    end 
    
    % calculate T for euler angle rate->angular velocity
    H = [1,0,-sin(x_old(5)); 
         0, cos(x_old(4)), sin(x_old(4))*cos(x_old(5));
         0, -sin(x_old(4)), cos(x_old(4))*cos(x_old(5))];
        
    T = cat(2, eye(3,3), zeros(3,3));
    temp = cat(2, zeros(3,3),H);
    T = cat(1,T, temp);
    
    
    % calculate Jg
    Jg = -D*J*T;
    
    delta_x = -(Jg\g)*0.001;
    
    x_new = x_old + delta_x;
    x_old = x_new;
    disp(x_old);
    if norm(g)<allowed_err
        break
    end 
end

