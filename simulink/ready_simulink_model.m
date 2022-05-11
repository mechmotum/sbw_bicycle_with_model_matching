clear;
clc;
close all;
start = tic;
BEGIN_ACADO;
%% Settings and parameters
EXPORT = 1;
COMPILE = 1;
Ts = 1/80;              % Sampling time 1/Hz [s]
Thorizon = 2;           % Time horizon [s]
Nhorizon = Thorizon/Ts; % Number of steps in the horizon [-]
bicycle_name = 'batavus_browser'; % Bicycle from 'bicycles/' folder
g = 9.81;               % Gravitational constant [m/s^2]
T_phi = 0;              % Generalised lean torque [Nm]

Tstart = 0;             % Start time of the reference [s]
Tend = 29;              % Length of one maneouvre  [s]
width = 0.5;            % Width of the lane-change [m]
slope = 0.02;            % Slope of the lane-change [-]
v = 5.5;                % Bicycle velocity during the experiment [m/s]

par = get_bikePars(bicycle_name);
[ M_pp, M_pd, M_dp, M_dd, K0_pp, K0_pd, K0_dp, K0_dd, ...
  K2_pd, K2_dd, C1_pd, C1_dp, C1_dd ] = get_matrices(par);
w = par.w;              % Wheel base [m]
c = par.c;              % Trail [m]
lambda = par.lambda;    % Steer axis tilt (pi/2 - head angle) [rad]

%% Dynamic model
% Differential states
DifferentialState x_P y_P psi;  % Rear wheel position [m], yaw [rad]
DifferentialState phi delta;    % Lean angle, steer angle [rad]
DifferentialState d_phi d_delta;% Lean rate, steer rate [rad/s]
Control T_delta;                % Handlebar/steer torque [Nm]
Disturbance W;
% Whipple-Carvallo model equations
f = acado.DifferentialEquation();
f.add( dot(x_P) == v * cos(psi) );
f.add( dot(y_P) == v * sin(psi) );
f.add( dot(psi) == ((v * delta + c * d_delta) / w) * cos(lambda) );
f.add( dot(phi) == d_phi );
f.add( dot(delta) == d_delta );
f.add( dot(d_phi) == (M_dd * T_phi - M_pd * T_delta - ...
    K2_pd * M_dd * delta * v^2 + K2_dd * M_pd * delta * v^2 - ...
    K0_pd * M_dd * delta * g + K0_dd * M_pd * delta * g - ...
    C1_pd * M_dd * d_delta * v + C1_dd * M_pd * d_delta * v + ...
    C1_dp * M_pd * d_phi * v - K0_pp * M_dd * g * phi + ...
    K0_dp * M_pd * g * phi) / (M_dd * M_pp - M_dp * M_pd) );
f.add( dot(d_delta) == -(M_dp * T_phi - M_pp * T_delta - ...
    K2_pd * M_dp * delta * v^2 + K2_dd * M_pp * delta * v^2 - ...
    K0_pd * M_dp * delta * g + K0_dd * M_pp * delta * g - ... 
    C1_pd * M_dp * d_delta * v + C1_dd * M_pp * d_delta * v + ...
    C1_dp * M_pp * d_phi * v - K0_pp * M_dp * g * phi + ...
    K0_dp * M_pp * g * phi) / (M_dd * M_pp - M_dp * M_pd) );
% Get the number of states and controls
Ndiff = length(diffStates);
Nctrl = length(controls);

%% MPC export
acadoSet('problemname','sbw_treadmill');
ocp = acado.OCP(0.0, Thorizon, Nhorizon);
% Cost function
h = [diffStates; controls];
hN = [diffStates];
W = acado.BMatrix(eye(Ndiff+Nctrl));
WN = acado.BMatrix(eye(Ndiff));
ocp.minimizeLSQ(W, h);
ocp.minimizeLSQEndTerm(WN, hN);
% Model and constraints
ocp.setModel(f);
ocp.subjectTo(deg2rad(-20) <= phi <= deg2rad(20));
ocp.subjectTo(deg2rad(-40) <= delta <= deg2rad(40));
ocp.subjectTo(-7 <= T_delta <= 7);
% Export options
mpc = acado.OCPexport(ocp);
mpc.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');
mpc.set('DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING');
mpc.set('SPARSE_QP_SOLUTION', 'FULL_CONDENSING_N2');
mpc.set('INTEGRATOR_TYPE', 'INT_IRK_GL4');
mpc.set('NUM_INTEGRATOR_STEPS', 2*Nhorizon);
mpc.set('QP_SOLVER', 'QP_QPOASES3');
mpc.set('HOTSTART_QP', 'YES');
mpc.set('LEVENBERG_MARQUARDT', 1e-4);
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');
mpc.set('GENERATE_MATLAB_INTERFACE', 'NO');

if EXPORT
    disp('---------------------------------------')
    disp('             EXPORTING MPC')
    disp('---------------------------------------')
    tic;
        mpc.exportCode('sbw_export');
    compTime = toc;
    disp(['Done. Time taken: ' num2str(compTime) 's'])
end

if COMPILE
    disp('---------------------------------------')
    disp('             COMPILING MPC')
    disp('---------------------------------------')
    tic;
        cd sbw_export
        make_acado_solver_sfunction
        %copyfile('acado_solver_sfun.mex*', '../')
        cd ..
    compTime = toc;
    disp(['Done. Time taken: ' num2str(compTime) 's'])
end
END_ACADO;

%% Initial MPC settings
disp('---------------------------------------')
disp('              INITIALISE')
disp('---------------------------------------')
tic;
    x0 = zeros(1, Ndiff);
    uref = zeros(Nhorizon,1);
    input.x = repmat(x0, Nhorizon+1, 1)';
    input.od = zeros(Nhorizon+1, 1);
    input.u = uref';
    input.y = [repmat(x0, Nhorizon, 1) uref]';
    input.yN = x0';
    input.W  = diag([0 1e5 0 0 1e4 1 10 100]); % STAGE COST 1e5
    input.WN = diag([0 0 0 0 0 0 0]); % TERMINAL COST
    input.x0 = x0.';
    init.x   = input.x(:).';
    init.u   = input.u(:).';
    init.y   = input.y(:).';
    init.yN  = input.yN(:).';
    init.W   = input.W(:).';
    init.WN  = input.WN(:).';
    init.x0  = input.x0(:).';
compTime = toc;
disp(['Done. Time taken: ' num2str(compTime) 's'])


%% Reference to track
disp('---------------------------------------')
disp('              FIND REFERENCE')
disp('---------------------------------------')
tic
    % Create a format string for the reference txt export
    str_one_val = ",%.3f";
    str_all_val = "%.3f";
    for i=1:Nhorizon-1
        str_all_val = str_all_val + str_one_val;
    end
    % Create a reference line
    lane_ref = generate_reference(Tstart, Tend+Thorizon, Ts, ...
                                    (width/slope)/v, v, width);
    % Create a stage reference for MPC, combine with lane change profile
    ref = zeros(length(lane_ref), 1+Ndiff+Nctrl);
    ref(:,1) = lane_ref(:,1); % First column is time
    ref(:,3) = lane_ref(:,3) - (width/2); % Third column is y_P
    % Reshape the whole horizon into one row for use in MPC
    acado_ref = zeros(length(ref)-Thorizon/Ts, 1+Nhorizon*(Ndiff+Nctrl));
    unity_enum = zeros(length(acado_ref),1);
    string_ref = "";
    for i=1:length(acado_ref)
        tempRef = ref(i:(i+Nhorizon-1),2:end);
        acado_ref(i,1) = lane_ref(i,1);
        acado_ref(i,2:end) = reshape(tempRef', 1, []);
        unity_enum(i) = i-1;
        string_ref(i,:) = sprintf(str_all_val, ...
            acado_ref(i,3:8:size(acado_ref,2)));
    end
    % Save the reference as a file for use in Unity
    fid = fopen('reference_strings.txt','w');
    for i=1:length(string_ref)
        fprintf(fid,'%s\n',string_ref(i));
    end
    fclose(fid);
    clear tempRef lane_ref ref; 
    clear fid string_ref str_one_val str_all_val acado_enum;
    % Create a structure with no time to allow for looping in Simulink
    % This lets us run Simulink indefinitely and have the reference line
    % be periodic while also reducing the amount of memory used.
    % 'acado_ref' array should only be used for simulation
    % 'ref_struct' structure should be used for treadmill trials
    ref_struct = struct();
    ref_struct.signals.values = [unity_enum, acado_ref(:,2:end)];
    ref_struct.time = {};
compTime = toc;
disp(['Done. Time taken: ' num2str(compTime) 's'])

%% Open model
open_system("sbw_treadmill.slx");
disp('---------------------------------------')
disp(['    SCRIPT COMPLETED IN ' num2str(toc(start)) 's'])
disp('---------------------------------------')