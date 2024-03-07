function [qInterp] = PlanTrajectory(startPos, targetPos, startOri, EndOri, mode, previosPos, previousOri, prevWaypoint)
        
    positionFromTarget = constraintPositionTarget('stage_body');
    positionFromTarget.ReferenceBody = 'base';
    positionFromTarget.PositionTolerance = 0.000000001;
    
    oriFromTarget = constraintOrientationTarget('stage_body');
    oriFromTarget.ReferenceBody = 'base';
    oriFromTarget.OrientationTolerance = 0.000000001;
    
    numWaypoints = 5;
    startingPosition = startPos;
    FinalPosition = targetPos;
    
    startingOri = startOri;
    FinalOri = EndOri;
    
    % initialization mode
    % move from home position
    q0;
    if mode == 0
        q0 = homeConfiguration(assem);
    else 
        q0 = gik1(qWaypoints(k-1,:), positionFromTarget, oriFromTarget, poseTgt1, poseTgt2,poseTgt3,poseTgt4,poseTgt5,strokeConstraint);
    end 

    Waypoints=repmat(q0, numWaypoints,1);
    position_values = repmat([0,0,0], numWaypoints-1,1);
    ori_values = repmat([0,0,0,0], numWaypoints-1,1);
    
    for i = 1:3
        position_values(:,i) = linspace(startingPosition(i), FinalPosition(i), numWaypoints-1)';
        ori_values(:,i) =  linspace(startingOri(i), FinalOri(i), numWaypoints-1)';
    end 
    
    
    for k= 2:numWaypoints
        positionFromTarget.TargetPosition = position_values(k-1,:);
        oriFromTarget.TargetOrientation = ori_values(k-1, :);
        [qWaypoints(k,:),solutionInfo] = gik1(qWaypoints(k-1,:), positionFromTarget, oriFromTarget, poseTgt1, poseTgt2,poseTgt3,poseTgt4,poseTgt5,strokeConstraint);
    end
                                              
    % Visualize generated trajectory
    % Interpolate between the waypoints to generate a smooth trajectory. Use pchip to avoid overshoots,
    % which might violate the joint limits of the robot.
    framerate = 5;
    r = rateControl(framerate);
    tFinal = 10;
    tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
    numFrames = tFinal*framerate;
    qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';
    
    stagePosition = zeros(numFrames,3);
    for k = 1:numFrames
     stagePosition(k,:) = tform2trvec(getTransform(assem,qInterp(k,:), ...
     "stage_body"));
    end 

end