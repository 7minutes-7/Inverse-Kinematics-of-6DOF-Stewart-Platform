%% Calculate x,y,z bound
x = [0,0,196.9]';
angles = [0,0,0];

buf_z = zeros(3,1000);
buf_xy = zeros(3,1000000);
buf_xy_edges = zeros(3,1000);
z_size=0;
xy_size=0;
xy_size_1 = 0;
dx = 0.5;

%% Calculating z bound

% Find mimimum z 
z = 200;
dz = 0.1;
while(true)
    try
        calc_motor_displacement(x,angles(1),angles(2),angles(3));
        z = z - dz;
        x(3) = z
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            x(3) = z + dz;
            break
        end 
    end
end 

% Find maxium z
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
%axis([0 50 0 50 195 215])
xlabel('x')
ylabel('y')
zlabel('z')
%view(90,0)
%saveas(f1, "Translation_yz.png")

%% Calculating tilting angles on each z bound
% Considering only one angle tilt (yaw/pitch/roll)
% 1. yaw
ddelta = 0.1;
buf_yaw = zeros(1,10000);
for i = 1:z_size
    x = buf_z(:,i);
    for j = 1:10000
        try
           calc_motor_displacement(x,angles(1),angles(2),angles(3));
           angles = angles + [ddelta,0,0]
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
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
buf_roll = zeros(1,10000);

% Make new z buffer for evaluating roll angles 
% (smaller delta(0.1) near maximum peak 203~204)

new_buf_z = [];
new_buf_z_size = z_size;
critical_z = 206.5;
for i = 1:z_size
    x = buf_z(3,i);
    if(x>critical_z)
        new_buf_z = buf_z(3,1:i);
        n = (buf_z(3,i+3)-new_buf_z(end))/0.1 - 2;
        new_buf_z_size = new_buf_z_size + n - 1;
        new_buf_z = cat(2, new_buf_z, new_buf_z(end)+0.1:0.1:buf_z(3,i+3)-0.1);
        new_buf_z = cat(2, new_buf_z, buf_z(3,i+3:end));
        break
    end
end
temp = zeros(2, size(new_buf_z,2));
new_buf_z = [temp ; new_buf_z];

for i = 1:new_buf_z_size
    x = new_buf_z(:,i); 
    for j = 1:10000
        try
           calc_motor_displacement(x,angles(1),angles(2),angles(3));
           % Check in smaller delta near the maximum peak
           if (x(3) > critical_z && x(3)<critical_z+1)
               ddelta = 0.01;
           else
               ddelta = 0.1;
           end
           angles = angles + [0,ddelta,0]
        catch ME
            if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
                buf_roll(i) = angles(2);
                break
            end
        end
    end 
    angles = [0,0,0];
end 

%{
for i = 1:z_size
    x = buf_z(:,i); 
    for j = 1:10000
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
%}

% plot results
figure(f2);
hold on;

%scatter(buf_z(3,1:z_size), buf_roll(1:z_size),".")

scatter(new_buf_z(3,1:new_buf_z_size), buf_roll(1:new_buf_z_size),".")
legend('yaw(z)','roll(x)')
%saveas(f2,"Rotation_plot.png")