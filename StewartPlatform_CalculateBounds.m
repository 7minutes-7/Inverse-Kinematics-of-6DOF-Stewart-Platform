%% Calculate x,y,z bound
x = [0,0,196.9]';
angles = [0,0,0];

buf_z = zeros(3,1000);
buf_xy = zeros(3,1000000);
z_size=0;
xy_size=0;
dx = 0.5;

%% Calculating z bound
for i = 1:1000
    try
        calc_motor_displacement(x,angles(1),angles(2),angles(3));
        buf_z(:,i) = x;
        x(3) = x(3) + dx;
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible'))
            sprintf("Max z is %f", x(3))
            break
        end 
    end
end 
z_size = i - 1 ;

%% Calculating x,y bound based on the z bound
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:1000
        for k = 1:1000
            try
                calc_motor_displacement(x,angles(1),angles(2),angles(3));
                
                xy_size = xy_size + 1;
                buf_xy(:,xy_size) = x;
                
                x(2) = x(2) + dx; 
            catch ME
                if(strcmp(ME.identifier, 'StewartPlatform:notPossible'))
                    %{
                    if(k~=1)
                        x(2) = x(2) - dx;
                        xy_size = xy_size + 1;
                        buf_xy(:,xy_size) = x;
                    end
                    %}
                    break
                end 
            end
        end 

        if(k==1) 
            break
        end 

        x(1) = x(1) + dx
        x(2) = 0;
    end 
end 


%% Plot results (symmetric on all 4 quadrants)
f1 = figure();
figure(f1);
scatter3(buf_xy(1,1:xy_size), buf_xy(2,1:xy_size), buf_xy(3,1:xy_size),1,'b')
%hold on;
%scatter3(buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), buf_xy(2,:), buf_xy(3,:),1,'b')
%hold on;
%scatter3(-buf_xy(1,:), -buf_xy(2,:), buf_xy(3,:),1,'b')
title('Range of translation at roatation [0,0,0]')
axis([0 50 0 50 195 215])
xlabel('x')
ylabel('y')
zlabel('z')


%% Calculating tilting angles on each z bound
% Considering only one angle tilt (yaw/pitch/roll)
% 1. yaw
ddelta = 0.1;
buf_yaw = zeros(1,1000);
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:1000
        try
           calc_motor_displacement(x,angles(1),angles(2),angles(3));
           angles = angles + [ddelta,0,0]
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible'))
                buf_yaw(i) = angles(1);
                break
            end
        end
    end 
    angles = [0,0,0];
end 

% plot results
f2 = figure();
figure(f2);
scatter(buf_z(3,1:z_size), buf_yaw(1:z_size),".")
title('Maximum angle at each z height')
xlabel('z coordinate')
ylabel('Angle')

% 2. roll
buf_roll = zeros(1,1000);
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:1000
        try
           calc_motor_displacement(x,angles(1),angles(2),angles(3));
           angles = angles + [0,ddelta,0]
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible'))
                buf_roll(i) = angles(2);
                break
            end
        end
    end 
    angles = [0,0,0];
end 

% plot results
figure(f2);
hold on;
scatter(buf_z(3,1:z_size), buf_roll(1:z_size),".")
legend('yaw(z)','roll(x)')

%% x,y range on different angles?
x = [0,0,203.4]';
angles = [0,0,0];


