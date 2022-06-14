clc; clear; close all; clear mex;
start = tic;
%% Settings and parameters
Ts_mpc = 1/80;          % Sampling time controller 1/Hz [s]
Ts_udp = 1/160;         % Sampling time UDP 1/Hz [s]
Ts_ser = 1/200;         % Sampling time serial 1/Hz [s]
Thorizon = 2;           % Time horizon [s]
nH = Thorizon/Ts_mpc; % Number of steps in the horizon [-]
% Bicycle settings
tracker_height = 0.9;   % Height of the tracker relative to ground [m]
steerTorqLim = 3;       % Steering torque limit [+- Nm]
fCutoff = 8;
% Constraints. Assumed symmetric. _x is the state, _u is the control
ub_x = [inf inf deg2rad(20) deg2rad(40) inf inf];
ub_u = [steerTorqLim];
lb_x = -ub_x;
lb_u = -ub_u;
% Cost function weights
%Q = diag([25 75 30 20 0 0]); % State
Q = diag([10 5 0 0 0 0]); % State
R = diag([1]); % Control
% Reference
width = 0.30; % Width of the lane-change
slope = 0.035; % Slope of the lane-change
v = 15/3.6; % Bicycle velocity during the experiment
Tchange = (width/slope)/v; % How much time the lane change takes
Tend = 20 + 2*Tchange; % Two lane changes 10s apart

%% Set up the dynamic system
load("rigidRiderStateSpace.mat"); % Load continuous model
ss_d = c2d(bicycle, Ts_mpc); % Translate to discrete model
A_ss = ss_d.A;
B_ss = ss_d.B;
clear bicycle;
% Get the number of states and controls
nX = size(A_ss,1);
nU = size(B_ss,2);

%% Build matrices
% Initialise
A_full = zeros(nX * (nH + 1), nX);
B_full = zeros(nX * nH, nU * nH);
Q_full = zeros(nX * (nH + 1));
R_full = zeros(nU * nH);
ub_x_full = zeros(nX * (nH + 1), 1);
ub_u_full = zeros(nU * nH, 1);
lb_x_full = ub_x_full;
lb_u_full = ub_u_full;
% Populate
for i=1:nH
    % A
    A_full(nX*(i-1)+1 : nX*i, :) = A_ss^(i-1);
    % B
    B_full(i*nX+1 : (i+1)*nX, (i-1)*nU+1 : i*nU) = B_ss;
    for j=1:(i-1)
        B_full(i*nX+1 : (i+1)*nX, (i-j-1)*nU+1 : (i-j)*nU) = A_ss^j*B_ss;
    end
    % Q
    Q_full((i-1)*nX+1 : i*nX, (i-1)*nX+1 : i*nX) = Q;
    % R
    R_full((i-1)*nU+1 : i*nU, (i-1)*nU+1 : i*nU) = R;
    % ub_x lb_x
    ub_x_full(nX*(i-1)+1 : nX*i, :) = ub_x;
    lb_x_full(nX*(i-1)+1 : nX*i, :) = lb_x;
    % ub_u lb_u
    ub_u_full(nU*(i-1)+1 : nU*i, :) = ub_u;
    lb_u_full(nU*(i-1)+1 : nU*i, :) = lb_u;
end
A_full(nX*nH+1 : nX*(nH+1), :) = A_ss^(nH);
Q_full(nX*nH+1 : nX*(nH+1), nX*nH+1 : nX*(nH+1)) = Q;
ub_x_full(nX*nH+1 : nX*(nH+1), :) = ub_x;
lb_x_full(nX*nH+1 : nX*(nH+1), :) = lb_x;


%% Calculate static QP matrices
F = B_full' * Q_full * A_full;
D = -B_full' * Q_full;

H = B_full' * Q_full * B_full + R_full;
A = B_full;
ub_qp = ub_u_full;
lb_qp = lb_u_full;

%% Find the reference trajectory
% Create a format string for the reference txt export
str_one_val = ",%.3f";
str_all_val = "%.3f";
for i=1:nH
    str_all_val = str_all_val + str_one_val;
end
% Create a reference line
lane_ref = generateReference(Tend+Thorizon+Ts_mpc, Ts_mpc, ...
                                Tchange, v, width);
% Create a stage reference for MPC, combine with lane change profile
ref = zeros(length(lane_ref), 1+nX);
ref(:,1) = lane_ref(:,1); % First column is time
ref(:,2) = lane_ref(:,3) - (width/2); % Second column is y_P
% Reshape the whole horizon into one row for use in MPC
acado_ref = zeros(length(ref)-Thorizon/Ts_mpc, 1+(nH+1)*nX);
unity_enum = zeros(length(acado_ref),1);
string_ref = "";
for i=1:length(acado_ref)
    tempRef = ref(i:(i+nH),2:end);
    acado_ref(i,1) = lane_ref(i,1);
    acado_ref(i,2:end) = reshape(tempRef', 1, []);
    unity_enum(i) = i-1;
    string_ref(i,:) = sprintf(str_all_val, ...
        acado_ref(i,2:nX:size(acado_ref,2)));
end
% Save the reference as a file for use in Unity
fid = fopen('reference_strings.txt','w');
for i=1:length(string_ref)
    fprintf(fid,'%s\n',string_ref(i));
end
fclose(fid);
%clear tempRef lane_ref ref; 
%clear fid string_ref str_one_val str_all_val acado_enum;
% Create a structure with no time to allow for looping in Simulink
% This lets us run Simulink indefinitely and have the reference line
% be periodic while also reducing the amount of memory used.
% 'acado_ref' array should only be used for simulation
% 'ref_struct' structure should be used for treadmill trials
ref_struct = struct();
ref_struct.signals.values = [unity_enum, acado_ref(:,2:end)];
ref_struct.time = {};

%% Run the simulation
open_system ("controllerModel.slx");
disp('---------------------------------------')
disp(['    SCRIPT COMPLETED IN ' num2str(toc(start)) 's'])
disp('---------------------------------------')