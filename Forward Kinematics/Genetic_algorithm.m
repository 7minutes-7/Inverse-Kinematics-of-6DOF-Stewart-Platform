

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
joint_rotation_P_left = 80;  % [deg]
joint_rotation_P_right = -10;  % [deg]
joint_rotation_B_left = 0; %[deg]
joint_rotation_B_right = 0; %[deg]
left_angle_shift_B = 6; %[deg]
right_angle_shift_B = 19; %[deg]
left_angle_shift_P = 20;
right_angle_shift_P = 20;
P_offset = 6.5; %[deg]

%% Calculate remaining constants from given input
joint_rotation_P_left = joint_rotation_P_left* pi/180; %[rad]
joint_rotation_P_right = joint_rotation_P_right* pi/180; %[rad]
joint_rotation_B_left = joint_rotation_B_left * pi/180; %[rad]
joint_rotation_B_right = joint_rotation_B_right * pi/180; %[rad]
joint_rotation_B = [joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right,joint_rotation_B_left, joint_rotation_B_right];
joint_rotation_P = [joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right,joint_rotation_P_left,joint_rotation_P_right];

om1 = left_angle_shift_B*pi / 180 ; % left angle shift base
om2 = right_angle_shift_B*pi/180; % right angle shift base
delta_base = [pi/6 - om1, pi/2 + om2, 5*pi/6-om1, 7*pi/6 + om2, 3*pi/2-om1, 11*pi/6+om2];


om3 = left_angle_shift_P*pi / 180 ; % left angle shift stage
om4 = right_angle_shift_P*pi/180; % right angle shift stage
delta_platform = P_offset * pi/180.0 + [pi/3-om3, pi/3 + om4, pi - om3, pi + om4, 5*pi/3 - om3, 5*pi/3 + om4];


radius_B = [radius_B_left, radius_B_right, radius_B_left, radius_B_right, radius_B_left, radius_B_right];

%% Input
% oa :Base joint coordinates OA
% cb: Platform joint coordinates CB
% L: Link length
oa = zeros(3,3);
cb = zeros(3,3);

init_guess_T = [0,0,120]';
init_guess_R = [0,0,0]';

theta_input = [7.75, 7.75, 7.75, 7.75, 7.75, 7.75]';

% Random distribution range for initial population selection
err_T = 0.0001; % [mm]
err_R = 0.001; % [deg]

ERR_TOLERANCE = 0.04;
%% Used variables
% Population size = 40
% Chromosome size = 9 (OB1, OB2, OB3)
P_SIZE = 40;
CH_SIZE = 6;
chromo = zeros(CH_SIZE, P_SIZE);
loss = zeros(1,P_SIZE); % array to store loss
q_arr = zeros(3,6);

B1B2 = 36.2541; % [mm]
B2B3 = 68.1355; % [mm]
B3B1 = 91.7987; % [mm]
 
chromo(1:3, :) = init_guess_T - err_T + 2* err_T * rand(3,P_SIZE);
chromo(4:6, :) = init_guess_R - err_R + 2* err_R * rand(3,P_SIZE);

%% Implement algorithm

current_guess_T = zeros(3,1);
current_guess_R = zeros(3,1);

% Start optimization iteration
while(true)
    % 1. Loss calculation
    loss = zeros(1,P_SIZE);
    for i=1:P_SIZE
        for j = 1:6
            current_guess_T = chromo(1:3,i);
            current_guess_R = chromo(4:6,i);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Inverse Kinematics
            % Reshape input to column matrix
            x = reshape(current_guess_T,[3,1]);
    
            % subtract offsets
            R_PtoB = calc_RMatrix(pi*current_guess_R(1)/180.0, pi*current_guess_R(2)/180.0, pi*current_guess_R(3)/180.0);
            x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
    
            p = radius_P * [cos(delta_platform(j)),sin(delta_platform(j)),0]'; % platform coordinates
            q = x + R_PtoB*p; % base coordinates
            mu = radius_B(j) * [cos(delta_base(j)), sin(delta_base(j)), 0]'; % base coordinates
            q_arr(:,j) = q; % for debugging purposes

            R_MuToB = calc_RMatrix(joint_rotation_B(j) + delta_base(j),0,0); % joint coordinates to base
            s = R_MuToB\(q - mu); % joint coordinates

            r = L*L - s(1)*s(1) - s(2)*s(2);
            t1 = s(3) + sqrt(r);
            t2 = s(3) - sqrt(r); 
        
            if(r<0)
               throw(ME1)
            elseif(t1>=0 && t1<=theta_max)
               theta = t1;
            elseif(t2>=0 && t2<=theta_max)
               theta = t2; % Always chosen
            else 
               throw(ME1)
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            loss(i) = loss(i) + norm(theta_input(j) - theta);
        end
        % 
        % loss(i) = loss(i) + (B1B2.^2 - (q_arr(1,1)-q_arr(1,2)).^2 + (q_arr(2,1)-q_arr(2,2)).^2 +(q_arr(3,1)-q_arr(3,2)).^2 ).^2 + ...
        %                     (B2B3.^2 - (q_arr(1,2)-q_arr(1,3)).^2 + (q_arr(2,2)-q_arr(2,3)).^2 +(q_arr(3,2)-q_arr(3,3)).^2 ).^2 + ...
        %                     (B3B1.^2 - (q_arr(1,3)-q_arr(1,1)).^2 + (q_arr(2,3)-q_arr(2,1)).^2 +(q_arr(3,3)-q_arr(3,1)).^2 ).^2;
    end
    
    
    % 2. Selection
    % 40C2
    % Elitist strategy
    [loss_sort, sort_idx] = sort(loss, "ascend");

    elite_idx = nchoosek(1:10,2);
    elite_comb = elite_idx(1:40,:); % combination of two elites
    
    % selection_idx = nchoosek(1:P_SIZE,2);
    % selection_idx = randi([1 size(selection_idx,1)],1,P_SIZE) ;
    
    % selection1 : elite_comb(i,1)
    % selection2 : elite_conb(i,2)

    disp(loss_sort(1)); % the smallest loss
    if(loss_sort(1)<ERR_TOLERANCE)
        disp(chromo(:,sort_idx(1)));
        break;
    end

    % 3. Crossover
    % Hx pivot is used, for the reason that it has the best outcome in the paper
    % Wright’s heuristic crossover operator is combined with Pivot Mutation. 
    % The crossover and mutation rates are 0.8 and 0.1, respectively.
    for i = 1:P_SIZE
        x1 = chromo(:,sort_idx(elite_comb(i,1)));
        x2 = chromo(:,sort_idx(elite_comb(i,2)));
        offspring = 0.6 * (x1-x2) + x1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Inverse Kinematics
        
        % while(true)
        %     offspring = rand * (x2-x1) + x1;
        %     loss_offspring = 0;
        %     for j= 1:6
        %         % Reshape input to column matrix
        %         x = reshape(offspring(1:3),[3,1]);
        % 
        %         % subtract offsets
        %         R_PtoB = calc_RMatrix(pi*offspring(4)/180.0, pi*offspring(5)/180.0, pi*offspring(6)/180.0);
        %         x = x - [0,0, joint_height]' - R_PtoB*[0,0,platform_thickness + joint_height]';
        % 
        %         p = radius_P * [cos(delta_platform(j)),sin(delta_platform(j)),0]'; % platform coordinates
        %         q = x + R_PtoB*p; % base coordinates
        %         mu = radius_B(j) * [cos(delta_base(j)), sin(delta_base(j)), 0]'; % base coordinates
        %         q_arr(:,j) = q; % for debugging purposes
        % 
        %         R_MuToB = calc_RMatrix(joint_rotation_B(j) + delta_base(j),0,0); % joint coordinates to base
        %         s = R_MuToB\(q - mu); % joint coordinates
        % 
        %         r = L*L - s(1)*s(1) - s(2)*s(2);
        %         t1 = s(3) + sqrt(r);
        %         t2 = s(3) - sqrt(r);
        % 
        %         if(r<0)
        %             throw(ME1)
        %         elseif(t1>=0 && t1<=theta_max)
        %             theta = t1;
        %         elseif(t2>=0 && t2<=theta_max)
        %             theta = t2; % Always chosen
        %         else
        %             throw(ME1)
        %         end
        % 
        %         loss_offspring = loss_offspring + (theta_input(j) - theta).^2;
        %     end
        %     if loss_offspring<loss(i)
        %         break;
        %     end 
        % end
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

       


        chromo(:,i) = offspring;
    end

    % 4. Mutation
end