%% initialization script

%   z = [rollrate_c, pitchrate_c, yawrate_c, thrust_c, thrust_previous, x, y, z, vx, vy, vz, roll, pitch, yaw]
%   u = [rollrate_c, pitchrate_c, yawrate_c, thrust_c, thrust_previous]
%   x = [x, y, z, vx, vy, vz, roll, pitch, yaw]  

% these variables are not changeable when running

%% System physical parameters
global pr

% realworld_test
% pr.mass   =   1.1273;
% pr.g      =   9.7936; 

% rotors_sim
pr.mass  =   0.745319;
pr.g     =   9.81;

% speed limit, hard constraint
pr.state.maxVx      = 2.0;          % m/s
pr.state.maxVy      = 2.0;          % m/s
pr.state.maxVz      = 2.0;          % m/s
pr.mapsize          = [20; 20; 5];  % m
% control input 

pr.input.maxRollRate   = deg2rad(90);        % rad/s maximum roll rate
pr.input.maxPitchRate  = deg2rad(90);        % rad/s maximum pitch rate
pr.input.maxYawRate    = deg2rad(90);        % rad/s maximum yawrate
pr.input.maxThrust     = 2.0*pr.g*pr.mass;   % m/s2, max thrust cmd
pr.input.minThrust     = 0.5*pr.g*pr.mass;   % m/s2, min thrust cmd

%% Problem dimensions
global model

model.N             =   20  ;                    % horizon length
model.dt            =   0.05;                    % time step
model.nin           =   8   ;                    % number of control inputs (u)
model.nx            =   9   ;                    % number of state variables (x)
model.nvar          =   model.nin + model.nx ;   % number of stage variables (z)  8+9 = 17
model.neq           =   13 ;                     % number of equality constraints 9+4 = 13
model.nh            =   30 ;                     % number of polyhedron constraints Ax <= b
model.npar          =   10 + model.nh*4 ;        % number of parameters

%% Indexing
global index
% in stage vector, each stage
index.z.all           =   1:model.nvar;
index.z.inputs        =   1:8  ;      % control input
index.z.pos           =   9:11  ;     % position, [x, y, z]
index.z.vel           =   12:14 ;     % velocity, [vx, vy, vz]
index.z.euler         =   15:17;      % euler angles, [roll, pitch, yaw]

% in state vector, each stage
index.x.pos           =   1:3 ;       % position, [x, y, z]
index.x.vel           =   4:6 ;       % velocity, [vx, vy, vz]
index.x.euler         =   7:9 ;       % euler angles, [roll, pitch, yaw]

% in parameter vector, problem, each stage
index.p.all           =   1:model.npar;
index.p.wayPoint      =   1:3 ;                           % reference path point
index.p.extForceAcc   =   4:6 ;                           % the external force
index.p.weights       =   7:9 ;                           % [w_wp, w_input, w_input_rate]
index.p.yaw           =   10  ;                           % the reference yaw
index.p.polyConstA    =   11:10+model.nh*3;               % 3*n 
index.p.polyConstb    =   11+model.nh*3:10+model.nh*4 ;   % 1*n    nmax = 30;
