function cost = mpc_objectiveN_normal(z, p)

    % define horizon terminal cost function for mpc

    global index pr                     % global index information
    
    %% obtaining necessary information
    % cost terms weights
    w_wp         =   p(index.p.weights(1));  % waypoint cost weight
    w_input      =   p(index.p.weights(2));  % control inputs cost weight
    w_input_rate =   p(index.p.weights(3));
    % ego mav 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_input   =   z(index.z.inputs);      % control input [roll_c, pitch_c, thrust_c]
    ego_ref     =   p(index.p.wayPoint);    % goal [xg, yg, zg]
    yaw_ref     =   p(index.p.yaw);
    ego_yaw     =   z(index.z.euler(3));    % current yaw [psi]


    %% waypoint cost
    Q_wp_pos    =   w_wp * diag([1.0; 1.0; 1.0]);
    cost_wp_pos =   (ego_ref(1:3) - ego_pos)' * Q_wp_pos  * (ego_ref(1:3) - ego_pos);
    % the cost of yaw tracking
    cost_wp_yaw =  12* w_wp * ( yaw_ref - ego_yaw)^2;
    % total navigation cost
    cost_wp = cost_wp_pos + cost_wp_yaw;   

    %% control input cost
    % normalize the input to [-1, 1] 
    ego_input_normalized = [ego_input(1)/pr.input.maxRollRate; ego_input(2)/pr.input.maxPitchRate; ...
                            ego_input(3)/pr.input.maxYawRate];
                      
                        
    Q_input = w_input * diag([1.0; 1.0; 1.0]); 
    
    
    % the cost
    cost_input = ego_input_normalized' * Q_input * ego_input_normalized;
    
    
    %% control input smoothness cost 
    Q_input_rate    =  w_input_rate * diag([1.0; 1.0; 1.0; 1.0]);
    
    cost_input_rate = (ego_input(1:4) - ego_input(5:8))' * Q_input_rate * (ego_input(1:4) - ego_input(5:8));
    %% combine all cost
    cost = cost_wp + cost_input + cost_input_rate;
    

end
