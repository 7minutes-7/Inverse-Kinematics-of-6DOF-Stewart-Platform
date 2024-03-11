clear
close all

%% Building robot
% Robot parameters
LINK_HEIGHT = 0.12727;
KUKA_BASE_HEIGHT = 0.13045;

% Load KUKA robot
robot =  Func_kuka_robot_build_dhparam(); 
only_kuka = Func_kuka_robot_build_dhparam(); % For IK calculation purposes

% Load stewart platform 
assem = Build_stewart_simple();
% Add stewart to KUKA end effector
addSubtree(robot, robot.Bodies{end}.Name, assem);


% Change home position for both rigidBodyTrees
robot.Bodies{1}.Joint.HomePosition = deg2rad(-60);
only_kuka.Bodies{1}.Joint.HomePosition = deg2rad(-60);

robot.Bodies{2}.Joint.HomePosition = deg2rad(30);
robot.Bodies{4}.Joint.HomePosition = deg2rad(90);
robot.Bodies{6}.Joint.HomePosition = deg2rad(-30);
robot.Bodies{13}.Joint.HomePosition = LINK_HEIGHT; % stage_rot_z

only_kuka.Bodies{2}.Joint.HomePosition = deg2rad(30);
only_kuka.Bodies{4}.Joint.HomePosition = deg2rad(90);
only_kuka.Bodies{6}.Joint.HomePosition = deg2rad(-30);


% joint parameters
config0 = homeConfiguration(robot);
config = config0;

kuka_config0 = homeConfiguration(only_kuka);
kuka_config = kuka_config0;


% Variables for solving kinematics

gik = generalizedInverseKinematics("RigidBodyTree",only_kuka,"ConstraintInputs", {"position", "orientation"});
gik.SolverParameters.MaxIterations = 20;

eePose = getTransform(only_kuka, kuka_config0, only_kuka.Bodies{end}.Name,"base");

positionFromTarget = constraintPositionTarget(only_kuka.Bodies{end}.Name);
positionFromTarget.TargetPosition = eePose(1:3,4)'; % intialize target position to starting end effector position
positionFromTarget.PositionTolerance = 0.00015; % Position accuracy of LBR MED 14 - 0.15mm

oriFromTarget = constraintOrientationTarget(only_kuka.Bodies{end}.Name);
oriFromTarget.TargetOrientation = rotm2quat(eePose(1:3,1:3)); % initialize target orientation
oriFromTarget.OrientationTolerance = 0.0001; 


% Fixed frames and constants

r = rateControl(10); % 200 [Hz]
dt = r.DesiredPeriod; 

T_BF_init = getTransform(only_kuka, kuka_config0, only_kuka.Bodies{end}.Name,"base"); % Pose of kuka end effector
T_BF0 = T_BF_init;

T_FC =  eul2tform([0,0,0]) * trvec2tform([0,0,0.13045]);

T_CD_init = eul2tform([0,0,0]) * trvec2tform([0,0,LINK_HEIGHT]); % stewart_base to stage (intial)
T_CD0 = T_CD_init;


%showdetails(robot);

%% Show the robot in its initial configuration
figure;
show(robot, config, 'PreservePlot', false, "Visuals","off","Frames","on");
xlim([-1,1.5]);
ylim([-1,1.5]);
zlim([-0.5, 1.5]);
view(0,90);
hold on
t = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOP START
%% 1. Matrix calculation 
reset(r);
while(true)
    tic;
    % Take input
    twist_input = [0,0,0,0,0,0.05]';
    alpha = 0; % Motion ratio of stewart (Kuka: 1-alpha)

    % calculation
    Mtwist_Db = Twist2Matrix(twist_input);

    T_CD = T_CD0 * (eye(4,4) + alpha * Mtwist_Db * dt) ;  % stewart transformation matrix
    T_CD0 = T_CD;

    T_FD = T_FC * T_CD; % stewart 움직임만 반영

    twist_Fs = MakeAdjointMatrix(T_FD)*(1-alpha) * Matrix2Twist(Mtwist_Db); % twist to move KUKA
    
    Mtwist_Fb = Twist2Matrix(twist_Fs);
    T_BF = T_BF0 * (eye(4,4) + Mtwist_Fb * dt);
    T_BF0 = T_BF;

    %% 2. Solve kinematics

    % Kuka
    % update next target from calculated transform
    positionFromTarget.TargetPosition = T_BF(1:3, 4);
    oriFromTarget.TargetOrientation = rotm2quat(T_BF(1:3, 1:3)); % target in quaternions
    
    % calc ik for KUKA
    [kuka_config, solutionInfo] = gik(kuka_config0, positionFromTarget,oriFromTarget);
    for n = 1: numel(kuka_config) %  7 if normal
        % Copy calculated configuration of KUKA to the whole robot
        config(n).JointPosition = kuka_config(n).JointPosition;
    end
    kuka_config0 = kuka_config;

    % Stewart platform
    eul_CD = rotm2eul(T_CD(1:3, 1:3)); %[rad]
    p_CD = T_CD(1:3, 4)*1000.0; % [mm]

    try
        calc_motor_displacement(p_CD, eul_CD(1), eul_CD(2), eul_CD(3));
    catch ME
        if(strcmp(ME.identifier, 'StewartPlatform:notPossible')|| strcmp(ME.identifier, 'StewartPlatform:linkCollision'))
            % No solution for IK, Do something here
            % break loop..?
        end
    end
    % Copy calculated configuration of stewart to the whole robot
    config(8).JointPosition = T_CD(1,4); % [m]
    config(9).JointPosition = T_CD(2,4);
    config(10).JointPosition = T_CD(3,4);
    config(11).JointPosition = eul_CD(1);
    config(12). JointPosition = eul_CD(2);
    config(13).JointPosition = eul_CD(3);

    %% 3. Display results

    show(robot, config, 'PreservePlot', false, "Visuals","off","Frames","on");
    
    toc;
    waitfor(r);
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold off;


