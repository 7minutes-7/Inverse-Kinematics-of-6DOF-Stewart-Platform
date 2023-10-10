% At diameter 20, height 20

% Points to analyze
x = [[0;0;20],[10;0;20],[-10;0;20],[0;10;20],[0;-10;20]];

% add offsets
dtheta = 1;
rotation_bounds = zeros(2,5);


%% Run once when parameters are changed
% Find mimimum z (our starting point)
z = 210;
dz = 0.1;
while(true)
    try
        calc_motor_displacement([0;0;z],0,0,0);
        z = z - dz
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            z = z + dz;
            break
        end 
    end
end 

trans = x;
trans(3,:) = trans(3,:)+z;% our starting point with offsets
%{
% Find minimum first
theta = [0,0,0]; % Starting angle
for i = 1:5
    while(true)
        try
            calc_motor_displacement(trans(:,i),theta(1), theta(2), theta(3));
            theta = theta - dtheta;
        catch ME 
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                rotation_bounds(1,i) = theta(1) + dtheta;
                break;
            end 
        end 
    end 
end 

% Find maximum
theta = [0,0,0]; % Starting angle
for i = 1:5
    while(true)
        try
            calc_motor_displacement(trans(:,i),theta(1),theta(2),theta(3));
            theta = theta + dtheta
        catch ME 
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                rotation_bounds(2,i) = theta(1) - dtheta;
                break;
            end 
        end 
    end 
end 

% print results
for i = 1:5
    disp("Rotation bounds at point")
    disp(x(:,i)')
    disp(rotation_bounds(:,i)');
    disp("")
end 

%}

%% Find all possible points (roll, pitch, yaw)
dtheta = 5;
theta_start = 80;

% variables to count error types
motor_err_cnt = 0;
link_err_cnt = 0;

% Seperate buffers for the 5 critical points
buf_angle_1 = [];
buf_angle_2 = [];
buf_angle_3 = [];
buf_angle_4 = [];
buf_angle_5 = [];

% Re-run this triple for-loop here to get results of different points 
roll = -theta_start;
pitch = -theta_start;
yaw = -theta_start;
for i = 1 : theta_start*2/dtheta
    for j = 1 : theta_start*2/dtheta
         for k = 1: theta_start*2/dtheta
             try
                 % Change numbers here to analyze a different point
                 calc_motor_displacement(trans(:,2), yaw, pitch, roll);
                 buf_angle_1 = [buf_angle_1 , [roll;pitch;yaw]];
             catch ME 
                 if(strcmp(ME.identifier, 'StewartPlatform:notPossible'))
                     motor_err_cnt = motor_err_cnt + 1;
                 elseif(strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                     link_err_cnt = link_err_cnt + 1;
                 end 
             end 
             roll = roll + dtheta;
          end 
          pitch = pitch + dtheta;
          roll = -theta_start;
    end 
    yaw = yaw + dtheta
    pitch = -theta_start;
end 

% Plot results 
% Change numbers here for different points
f2 = figure();
figure(f2);
scatter3(buf_angle_1(1,:), buf_angle_1(2,:), buf_angle_1(3,:),1,'b')
title('Range of rotation at p1=[0,0,20]')
xlabel('roll')
ylabel('pitch')
zlabel('yaw')

