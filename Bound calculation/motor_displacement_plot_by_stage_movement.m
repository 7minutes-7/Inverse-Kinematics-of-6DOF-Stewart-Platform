%% Calculate x,y,z bound

angles = [0,0,0];

buf_size = 1000;
buf_z = zeros(3,buf_size);
buf_theta = zeros(7,buf_size); %[phi,theta1 ... theta6]'
z_size=0;
theta_size = 0;




%% Robot parameters
joint_thickness = 10.5; 

L = 50.6 + joint_thickness;
radius_P =53;
radius_B_left = 84;
radius_B_right = 84;
theta_max = 30; 
joint_height = 17.5;
platform_thickness = 7.0;
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


%% Last variables before VCM change
% L =80.6 + 10;
% radius_P = 25;
% radius_B_left = 65.8676832;
% radius_B_right = 56.25;
% theta_max = 30;
% joint_height = 14;
% platform_thickness = 6;
% joint_thickness = 10;
% base_joint_radius = 17.573379/2;
% stage_joint_radius = 16.620867/2;
% joint_rotation_P = 60;
% joint_rotation_B_left = 145.49430212;
% joint_rotation_B_right = 39.17;
% left_angle_shift_B = 0; %[deg]
% right_angle_shift_B = 0; %[deg]
% left_angle_shift_P = 6.5582618;
% right_angle_shift_P = 14.69;

%% Calculating z bound

%% Calculating z bound

% Find mimimum z 
z = 100;
x = [0,0,z]';
dz = 0.1;
while(true)
    try
        calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
        z = z - dz;
        x(3) = z;
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            x(3) = z + dz;
            sprintf("Min z is %f", x(3))
            break
        end 
    end
end 

% Find maxium z
for i = 1:buf_size
    try
        calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
        buf_z(:,i) = x;
        x(3) = x(3) + dx;
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            sprintf("Max z is %f", x(3))
            break
        end 
    end
end 
z_size = i - 1 ;
z_origin = buf_z(3,1);

%% Start displacment calculation
z_start = (buf_z(3,1) + buf_z(3,z_size))/2.0; % set z coordinate to the middle of ROM 
phi = 0; % [rad] , start coordinates [0,0, height_ROM/2]
r = 0.1; %[mm]


dphi= 0.1;

i = 1;
while(phi<=2*pi)
    x = r*cos(phi);
    y = r*sin(phi);
    
    trans = [x,y,z_start]';
    angles = [0,0,0]';

    try
        theta = calc_motor_displacement(trans,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                        joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                        left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                        left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                        base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                        joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                        L=L);

        % store calculated displacements to buffer
        buf_theta(1,i) = phi;
        buf_theta(2:end,i) = theta;

        % increment phi and indices
        phi = phi + dphi;
        i = i + 1;
        theta_size = theta_size + 1;
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            disp("impossible configuration found")
            return
        end 
    end
end


%% Plot results 
% x axis - phi
% y axis - displacment

x_axis = buf_theta(1,1:theta_size);

f1 = figure();
t = tiledlayout('flow');



% First subplot
subplot(2,1,1);
color2 = ["#7E2F8E", "#77AC30","#4DBEEE"];
for i = 1:3
    figure(f1);
    hold on
    plot(x_axis, buf_theta(i*2+1,1:theta_size), 'Color',color2(i),'LineWidth',2.0)
end 
legend('motor2','motor4','motor6')
xlabel('\phi [rad] ')
ylabel('Motor displacement [mm]')

xticks([0, 0.5*pi, pi, 1.5*pi, 2*pi])
xticklabels({'0','0.5\pi','\pi','1.5\pi','2\pi'})

% Second subplot
subplot(2,1,2);
color1 = ["#0072BD","#D95319","#EDB120"];
for i = 1:3
    figure(f1);
    hold on
    plot(x_axis, buf_theta(i*2,1:theta_size), 'Color', color1(i),'LineWidth',2.0)
end 
legend('motor1','motor3','motor5')
xlabel('\phi [rad] ')
ylabel('Motor displacement [mm]')

xticks([0, 0.5*pi, pi, 1.5*pi, 2*pi])
xticklabels({'0','0.5\pi','\pi','1.5\pi','2\pi'})


sgtitle("Stage moving at height " + num2str(buf_z(3,z_size) - z_start) + "mm, radius " + r + "mm" , 'FontSize', 20)

a = buf_theta;
a(a==0) = inf;
end_to_end_odd = max(buf_theta(2,:))-min(a(2,:))
end_to_end_even = max(buf_theta(3,:))-min(a(3,:))


