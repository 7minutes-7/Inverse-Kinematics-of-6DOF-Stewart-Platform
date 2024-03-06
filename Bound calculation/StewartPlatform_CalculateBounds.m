%% Calculate x,y,z bound

angles = [0,0,0];

buf_size = 1000;
buf_z = zeros(3,buf_size);
buf_xy = zeros(3,buf_size*buf_size);
buf_xy_edges = zeros(3,buf_size);
z_size=0;
xy_size=0;
xy_size_1 = 0;
dx = 0.5;

%% Robot parameters
joint_thickness = 10.5; 

L = 70.6 + joint_thickness;
radius_P =53;
radius_B_left = 84;
radius_B_right = 84;
theta_max = 30; 
joint_height = 17.5;
platform_thickness = 7.0;
link_radius = 3;
base_joint_radius = 19.723083/2;
stage_joint_radius = 19.723083/2;
joint_rotation_P_left = 10;  % [deg]
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

% Find mimimum z 
z = 130;
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

%% Calculating x,y bound based on the z bound
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:buf_size
        for k = 1:buf_size
            try
                calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                            joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                            left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                            left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                            base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                            joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                            L=L);
                xy_size = xy_size + 1;
                buf_xy(:,xy_size) = x;
                
                x(2) = x(2) + dx; 
            catch ME
                if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                    if(k~=1)
                        x(2) = x(2) - dx;
                        xy_size_1 = xy_size_1 + 1;
                        buf_xy_edges(:,xy_size_1) = x;
                    end
                    
                    break
                end 
            end
        end 

        if(k==1) 
            break
        end 

        x(1) = x(1) + dx;
        x(2) = 0;
    end 
end 


%% Plot results (symmetric on all 4 quadrants)
f1 = figure();
figure(f1);

scatter3(buf_xy(1,1:xy_size), buf_xy(2,1:xy_size), buf_xy(3,1:xy_size)-z_origin,1,'b')
%hold on;
%scatter3(buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
title('Range of translation at rotation [0,0,0]')
xlabel('x')
ylabel('y')
zlabel('z')
%view(90,0)
%saveas(f1, "Translation_yz.png")

%% Plot results (symmetric on all 4 quadrants)
f3 = figure();
figure(f3);

scatter3(buf_xy_edges(1,1:xy_size_1), buf_xy_edges(2,1:xy_size_1), buf_xy_edges(3,1:xy_size_1)-z_origin,1,'b')
%hold on;
%scatter3(buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
title('Edge range of translation at rotation [0,0,0]')
xlabel('x')
ylabel('y')
zlabel('z')
%view(90,0)
%saveas(f1, "Translation_yz.png")

%% Calculating tilting angles on each z bound
% Considering only one angle tilt (yaw/pitch/roll)
%% 1. yaw
ddelta = 0.1;
buf_yaw = zeros(1,10000);
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:10000
        try
            result = calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
            angles = angles + [ddelta,0,0];
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                buf_yaw(i) = angles(1) - ddelta;
                break
            end
        end
    end 
    angles = [0,0,0];
end 

% plot results
f2 = figure();
figure(f2);
scatter(buf_z(3,1:z_size)-z_origin, buf_yaw(1:z_size),".")
title('Maximum angle at each z height')
xlabel('z coordinate')
ylabel('Angle')
hold on;


%% 2. pitch
buf_pitch = zeros(1,10000);

for i = 1:z_size
    x = buf_z(:,i); 
    for j = 1:10000
        try
            calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
            angles = angles + [0,ddelta,0];
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                buf_pitch(i) = angles(2) - ddelta;
                break
            end
        end
    end 
    angles = [0,0,0];
end 

% plot results
scatter(buf_z(3,1:z_size)-z_origin, buf_pitch(1:z_size),".")
%}
%% 3. roll
buf_roll = zeros(1,10000);


for i = 1:z_size
    x = buf_z(:,i); 
    for j = 1:10000
        try
            calc_motor_displacement(x,angles(1),angles(2),angles(3), radius_P=radius_P, radius_B_left=radius_B_left, radius_B_right=radius_B_right,...
                                    joint_rotation_P_left=joint_rotation_P_left, joint_rotation_P_right=joint_rotation_P_right,joint_rotation_B_left = joint_rotation_B_left, joint_rotation_B_right = joint_rotation_B_right, ...
                                    left_angle_shift_B = left_angle_shift_B, right_angle_shift_B = right_angle_shift_B, ...
                                    left_angle_shift_P = left_angle_shift_P, right_angle_shift_P = right_angle_shift_P, ...
                                    base_joint_radius = base_joint_radius, stage_joint_radius= stage_joint_radius, ...
                                    joint_height= joint_height, joint_thickness= joint_thickness, platform_thickness= platform_thickness, ...
                                    L=L);
            angles = angles + [0,0,ddelta];
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                buf_roll(i) = angles(3)- ddelta;
                break
            end
        end
    end 
    angles = [0,0,0];
end 

% plot results
scatter(buf_z(3,1:z_size)-z_origin, buf_roll(1:z_size),".")
legend('yaw(z)','pitch(y)','roll(x)')
