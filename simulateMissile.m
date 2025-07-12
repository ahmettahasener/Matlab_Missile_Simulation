function [time, position, velocity, acceleration, dragForceMagnitude, hitTarget, finalDistanceToTarget] = simulateMissile(params)
% simulateMissile Simulates missile motion.
% This function simulates the flight of the missile according to given parameters.
% and returns data such as time, position, speed, acceleration, and drag force.

    initialPosition = params.initialPosition; % [x0, y0, z0] metre
    initialVelocityMagnitude = params.initialVelocityMagnitude; % m/s
    elevationAngleDeg = params.elevationAngleDeg; % Degrees
    azimuthAngleDeg = params.azimuthAngleDeg; % Degrees
    initialMass = params.initialMass; % kg
    dryMass = params.dryMass; % kg
    avgThrust = params.avgThrust; % N
    burnTime = params.burnTime; % s
    fuelFlowRate = params.fuelFlowRate; % kg/s
    referenceArea = params.referenceArea; % m^2
    dragCoefficient = params.dragCoefficient; % Cd
    targetPosition = params.targetPosition; % [xt, yt, zt] metre
    dt = params.dt; % Time step (s)
    
    % --- Constant---
    g = 9.81; % Gravitational acceleration (m/s^2)
    
    % --- Initial Conditions ---
    elevationAngleRad = deg2rad(elevationAngleDeg);
    azimuthAngleRad = deg2rad(azimuthAngleDeg); % Convert azimuth to radians
    
    initialVelocity = [initialVelocityMagnitude * cos(elevationAngleRad) * cos(azimuthAngleRad), ... % Vx
                       initialVelocityMagnitude * cos(elevationAngleRad) * sin(azimuthAngleRad), ... % Vy
                       initialVelocityMagnitude * sin(elevationAngleRad)];   
    
    
    % Lists for data storage
    time = 0;
    position = initialPosition;
    velocity = initialVelocity;
    acceleration = [0, 0, -g]; % Initially only gravity or acceleration at the moment of launch
    dragForceMagnitude = 0;
    
    currentMass = initialMass;
    currentTime = 0;
    currentPosition = initialPosition;
    currentVelocity = initialVelocity;
    
    % --- Simulation Cycle ---
    maxSimTime = 1000; % Maximum simulation time (does not go to infinity if no solution is found)
    
    while currentPosition(3) >= 0 && currentTime < maxSimTime % Until the Z position drops below 0 or until the max time
        
        % Update time
        currentTime = currentTime + dt;
        
        % Mass and Thrust Update
        thrustVector = [0, 0, 0]; % Default: no thrust
        if currentTime <= burnTime && currentMass > dryMass
            % Mass decreases during fuel burn
            currentMass = max(dryMass, initialMass - fuelFlowRate * currentTime);
            
            % Thrust force is applied parallel to the velocity direction (or simply vertical)
            % Here, as a simplified model, we can assume thrust is in the initial
            % launch direction or purely vertical. However, for a more realistic
            % approach, we should multiply by the normalized velocity vector.
            if norm(currentVelocity) > 0
                 thrustDirection = currentVelocity / norm(currentVelocity);
                 thrustVector = avgThrust * thrustDirection;
            else % If initial velocity is 0, use the initial launch direction
                 thrustDirection = [cos(elevationAngleRad) * cos(azimuthAngleRad), ...
                                    cos(elevationAngleRad) * sin(azimuthAngleRad), ...
                                    sin(elevationAngleRad)];
                 thrustVector = avgThrust * thrustDirection;
            end
        end
        
        % Air Density (Simplified Standard Atmosphere Model)
        % rho is calculated based on altitude (currentPosition(3))
        rho = standardAtmosphere(currentPosition(3)); 
        
        % Drag Force
        velocityMagnitude = norm(currentVelocity);
        dragForceMagnitude_instant = 0.5 * rho * dragCoefficient * referenceArea * velocityMagnitude^2;
        
        % Drag force acts in the opposite direction of the velocity vector
        dragVector = [0, 0, 0];
        if velocityMagnitude > 0
            dragVector = -dragForceMagnitude_instant * (currentVelocity / velocityMagnitude);
        end
        
        % Gravity Force
        gravityVector = [0, 0, -currentMass * g];
        
        % Net Force
        netForce = thrustVector + dragVector + gravityVector;
        
        % Acceleration
        currentAcceleration = netForce / currentMass;
        
        % Velocity Update (Euler Method)
        currentVelocity = currentVelocity + currentAcceleration * dt;
        
        % Position Update (Euler Method)
        currentPosition = currentPosition + currentVelocity * dt;
        
        % Save data
        time = [time; currentTime];
        position = [position; currentPosition];
        velocity = [velocity; currentVelocity];
        acceleration = [acceleration; currentAcceleration];
        dragForceMagnitude = [dragForceMagnitude; dragForceMagnitude_instant];
        
        % Check if it hit the ground or exceeded max height
        if currentPosition(3) < 0 && size(position,1) > 1 % If it dropped below 0, interpolate the last step to 0 altitude
            % Estimate the time and position at ground impact using the last two points
            t_prev = time(end-1);
            p_prev = position(end-1,:);
            v_prev = velocity(end-1,:);
            % Linearly interpolate to find the time and position at z = 0
            alpha = -p_prev(3) / (currentPosition(3) - p_prev(3));
            time(end) = t_prev + alpha * dt;
            position(end,:) = p_prev + alpha * (currentPosition - p_prev);
            velocity(end,:) = v_prev + alpha * (currentVelocity - v_prev);
            acceleration(end,:) = currentAcceleration; % Keep the last value for acceleration
            dragForceMagnitude(end) = dragForceMagnitude_instant; % Also for drag
            
            % Break the loop
            break; 
        end
    end
    
    % --- Target Control ---
    finalPosition = position(end,:);
    finalDistanceToTarget = norm(finalPosition - targetPosition);
    
    % Tolerance of reaching the target (e.g. within 50 meters)
    targetTolerance = 50; % metre
    hitTarget = finalDistanceToTarget <= targetTolerance;

end