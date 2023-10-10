%% Input
% x: translation
% alpha, beta, gamma : Euler angles of the platform relative to the base


%% INPUT VARIABLES
x = zeros(3,1);
angles = zeros(3,1);


%% Calculate Inverse Kinematics
while(true)
    % Take input
    x = input('Enter translation vector [x,y,z] or to exit, enter -1: ')';
    if(x == -1)
        disp('Exiting program...')
        return;
    end

    angles = input('Enter angles of rotation [alpha(z), beta(y), gamma(x)]: ')';

    try
        theta = calc_motor_displacement(x,angles(1),angles(2),angles(3));
        % Output results
        disp('Motor displacement:')
        disp(theta)
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible') || strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            disp(ME.message)

        end 
    end
end