function cost = mpc_objectiveN_final(z, p)

    % define horizon terminal cost function for mpc

    global index pr                     % global index information
    
    %% obtaining necessary information
    % cost terms weights
    w_wp         =   p(index.p.weights(1));  % waypoint cost weight
    w_input      =   p(index.p.weights(2));  % control inputs cost weight
    w_input_rate =   p(index.p.weights(3));
    % ego mav 
    ego_pos     =   z(index.z.pos);         % current stage position
    ego_input   =   z(index.z.inputs);      % control input
    ego_ref     =   p(index.p.wayPoint);    % reference point
    yaw_ref     =   p(index.p.yaw);
    ego_yaw     =   z(index.z.euler(3));    % current yaw [psi]
    ego_vel     =   z(index.z.vel);
    
    %% waypoint cost
    Q_wp_pos    =   w_wp * diag([1.0; 1.0; 1.0]);
    cost_wp_pos =   (ego_ref(1:3) - ego_pos)' * Q_wp_pos  * (ego_ref(1:3) - ego_pos);
    % the cost of yaw tracking
    cost_wp_yaw =    12* w_wp * ( yaw_ref - ego_yaw)^2;
    
    cost_wp_vel =  10* ego_vel'  *  Q_wp_pos * ego_vel ;    
    
%      % total navigation cost
    cost_wp = cost_wp_pos + cost_wp_yaw + cost_wp_vel;   

    
    % the cost of velocity
%     Q_wp_vel    =   w_wp * diag([1.0; 1.0; 1.0]);   
%     cost_vel = 0.5*(vel_ref(1:3) - ego_vel)' * Q_wp_vel ...
%         * (vel_ref(1:3) - ego_vel);
%     
%     % total navigation cost
%     cost_wp = cost_wp_pos + cost_wp_yaw + cost_vel;   
% 
%    
    %% control input cost
    % normalize the input to [-1, 1]
    
%     mid = (pr.input.maxThrust + pr.input.minThrust)/2;
    
%     ego_input_normalized = [ego_input(1)/pr.input.maxRollRate; ego_input(2)/pr.input.maxPitchRate; ...
%                             ego_input(3)/pr.input.maxYawRate;  ego_input(4)*1.0/pr.input.maxThrust ;];
%  
%     ego_input(3) =   eval_yawrate(ego_input(3));
    
    
    ego_input_normalized = [ego_input(1)/pr.input.maxRollRate; ego_input(2)/pr.input.maxPitchRate; ...
                            ego_input(3)/pr.input.maxYawRate];
                      
                        
    Q_input = w_input * diag([1.0; 1.0; 1.0]); 
    
    
    % the cost
    cost_input = ego_input_normalized' * Q_input * ego_input_normalized;
    
    
    %% control input smoothness cost 
    Q_input_rate    =  w_input_rate * diag([1.0; 1.0; 1.0; 1.0]);
    
%     ego_input(7) =   eval_yawrate(ego_input(7));
    
    
    cost_input_rate = (ego_input(1:4) - ego_input(5:8))' * Q_input_rate * (ego_input(1:4) - ego_input(5:8));
    %% combine all cost
    cost = cost_wp + cost_input + cost_input_rate;
    

end
