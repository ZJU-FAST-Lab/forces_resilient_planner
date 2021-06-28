%% Equality constraints
model.eq = @(z, p)transit(z,p);

model.E = [zeros(model.nx, model.nin), eye(model.nx);
           zeros(4, 4) eye(4), zeros(4, model.nx)];
    
%% Corridor constraints

% general nonlinear inequalities hl <= h(x) <=hu
model.ineq = @(z, p) mpc_corridorconst(z, p);

% upper/lower bound for inequalities
model.hl = -inf*ones(model.nh, 1);
model.hu = 0.00001*ones(model.nh, 1);


%% Objective function

model.objective{1} = @(z, p)mpc_objective1(z, p);

for i = 2: model.N -1
   model.objective{i}= @(z, p)mpc_objective_normal(z, p);
 end

model.objective{model.N} = @(z, p)mpc_objectiveN_normal(z, p);

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
model.lb = zeros(1, model.nvar);
model.ub = zeros(1, model.nvar);

% control input bound
model.lb(index.z.inputs) = [-pr.input.maxRollRate, -pr.input.maxPitchRate, -pr.input.maxYawRate, pr.input.minThrust, -pr.input.maxRollRate, -pr.input.maxPitchRate, -pr.input.maxYawRate, pr.input.minThrust];
model.ub(index.z.inputs) = [ pr.input.maxRollRate,  pr.input.maxPitchRate,  pr.input.maxYawRate, pr.input.maxThrust,  pr.input.maxRollRate,  pr.input.maxPitchRate,  pr.input.maxYawRate, pr.input.maxThrust];

% position should be in the map
model.lb(index.z.pos) = [-pr.mapsize(1), -pr.mapsize(2), 0.0];
model.ub(index.z.pos) = [ pr.mapsize(1),  pr.mapsize(2),  pr.mapsize(3)];

% velocity is hardly constrained
model.lb(index.z.vel) = [-pr.state.maxVx, -pr.state.maxVy, -pr.state.maxVz];
model.ub(index.z.vel) = [ pr.state.maxVx,  pr.state.maxVy,  pr.state.maxVz];

% euler angles are constrained
model.lb(index.z.euler) = [-0.4*pi, -0.4*pi, -2*pi];
model.ub(index.z.euler) = [ 0.4*pi,  0.4*pi,  2*pi];

%% Initial and final conditions
% here we only need to spercify on which variables initial conditions are imposed
model.xinitidx = [index.z.pos, index.z.vel, index.z.euler];
%% Define solver options
solver_name = strcat('FORCESNLPsolver_normal');
codeoptions = getOptions(solver_name);

codeoptions.platform    = 'Generic';% target platform
codeoptions.maxit       = 200;      % maximum number of iterations
codeoptions.printlevel  = 1;        % use printlevel = 2 to print progress
                                    % (but not for timings)
codeoptions.optlevel    = 3;        % 0: no optimization, 
                                    % 1: optimize for size, 
                                    % 2: optimize for speed, 
                                    % 3: optimize for size & speed
codeoptions.overwrite   = 1;
codeoptions.BuildSimulinkBlock = 0;
codeoptions.cleanup     = 0;
codeoptions.timing      = 1;
codeoptions.parallel    = 1;        % run prediction on multiple cores 
                                    % (better for longer horizons)
codeoptions.threadSafeStorage   = true; % the generated solver can be run
                                        % in parallel on different threads
codeoptions.BuildSimulinkBlock  = 0;% skipping builing of simulink S-function
codeoptions.nlp.linear_solver   = 'symm_indefinite_fast'; 
                                    % linear system solver, better for long horizons
codeoptions.nlp.checkFunctions  = 1;% not check the output of the function evaluations
codeoptions.noVariableElimination = 1;
codeoptions.nlp.TolStat = 1E-4;     % infinity norm tolerance on stationarity
codeoptions.nlp.TolEq   = 1E-4;     % infinity norm of residual for equalities
codeoptions.nlp.TolIneq = 1E-4;     % infinity norm of residual for inequalities
codeoptions.nlp.TolComp = 1E-4;     % tolerance on complementarity conditions

%% Generate FORCES PRO solver
fprintf('[%s] Normal Generating new FORCES solver...\n',datestr(now,'HH:MM:SS'));
FORCES_NLP(model, codeoptions);
fprintf('[%s] Normal FORCES solver generated OK \n',datestr(now,'HH:MM:SS'));

%% Storing the solver
% create a folder
folder_name = '../solver/normal';
files_name =  './matlab_solver/normal';

rmdir(folder_name, 's');
mkdir(folder_name);

rmdir(files_name, 's');
mkdir(files_name);

% move the generated solver to the folder
movefile('*.c', folder_name);
movefile('*.h', folder_name);
movefile(solver_name, folder_name); 
movefile('*forces', files_name);
movefile([solver_name, '*'], files_name);

% include path
addpath(genpath(pwd));
