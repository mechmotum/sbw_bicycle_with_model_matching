%% README
% This script runs the simulation of the MPC controller.
% 
% The same weights and reference are used as in the pilot study.
%
% The script outputs the "simulation_result.mat" file into "../data/" folder.
% 
% Written by S. Drauksas, 2022

clear;
clc;
addpath("functions/","qp/","mat_files/")

%% Load options
if ~isfile("opts.mat")
    generateOptionsFile();
end
load("opts.mat");

%% Load bicycle model
load("bike_ss.mat");
ss_d = c2d(bicycle, opts.Ts_mpc); % Convert from continuous to discrete
clear bicycle;

%% Load gate sequence
if ~isfile("structs.mat")
    generateReference(opts);
end
load("structs.mat");

%% Load the matrices needed for MPC/QP
if ~isfile("matrices.mat")
    generateMatrices(opts, ss_d);
end
load("matrices.mat");

%% Set additional options
opts.lookupBreakpoints = 1:8;
opts.lookupValues = [0, 0, ones(1, 6)];

%% Run Simulation 
% Simulate 480 seconds (2min no controller + 6min with controller)
out = sim("simulateController.slx");
save("../data/simulation_result.mat", "out");

%% Clean up
clear A_full B_full C_full lb_u_full lb_x_full R_full_zeros R_full;
clear blockCounterStruct opts QstructInitial Qstruct simulinkStruct;
clear simulinkStructInitial ss_d ub_x_full ub_u_full;
rmpath("functions/","qp/","mat_files/")
disp("Done!");