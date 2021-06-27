function dx = nonlinear_dynamics(x, u, p)
  
    global index pr
    
    %% control inputs
    rollrate_c  =   u(1);
    pitchrate_c =   u(2);
    yawrate_c   =   u(3); 
    thrust_c    =   u(4);

    %% velocities
    vx          =   x(4);
    vy          =   x(5);
    vz          =   x(6);
    
    %% euler angle: roll pitch yaw
    roll        =   x(7);
    pitch       =   x(8);
    yaw         =   x(9);
    
    R = [cos(yaw)*cos(pitch)  cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)  (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)); ...
    cos(pitch)*sin(yaw)  cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)  (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)); ...
    -sin(pitch)                            cos(pitch)*sin(roll)                            cos(pitch)*cos(roll)];

    z_B = R(:,3);
    
    linear_drag_coefficient_matrix = diag([0.33; 0.33; 0.00]);
    
    velocity = [vx; vy; vz];
    drag_acc = R * linear_drag_coefficient_matrix * R.'* velocity;
    
    acc = z_B*thrust_c/pr.mass + p(index.p.extForceAcc(1:3)) -[0.00; 0; pr.g] - drag_acc;
        

    droll   = rollrate_c;
    dpitch  = pitchrate_c;
    dyaw    = yawrate_c;

    %% output
    dx = [vx; vy; vz; acc; droll; dpitch; dyaw];


end
