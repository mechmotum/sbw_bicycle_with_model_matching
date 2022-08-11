clear;
clc;
addpath("functions\","qp\","mat_files\")

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

%% Load settings for the Familiarization + Baseline test
opts = readyFamiliarisation(opts);

%% Open Simulink
open_system("getAngleBias.slx");
calib = sim("getAngleBias.slx");
close_system("getAngleBias.slx");
open_system("mpcController.slx");
