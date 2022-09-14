%% README
% This script iterates through participants' data, extracts data from .mat and
% outputs .csv files.
% 
% Data is read from the "data/" folder and the .csv files are saved there as
% well.
%
% Trial consisting of BL and T1 should be called "result_t1.mat"
% Trial consisting of MTR and T2 should be called "result_t2.mat"
% Trial consisting of ETR should be called "result_ret.mat"
% Data from the no-hands test should be called "result_no_hands.mat"
% Simulation data should be called "result_sim.mat"
%
% This script uses two helper functions "csvData.m" and "csvSimData.m", which
% need to be located in the "functions/" folder.
%
% Written by S. Drauksas, 2022

clear;
clc;
addpath("functions/");

%% Exctract participant data
% Iterate through participants
for j = 1:10 
    disp("Participant " + num2str(j));
    fileName = "data" + filesep + "participant" + num2str(j) + "_";

    % Iterate through trials
    for i = ["result_t1.mat", "result_t2.mat", "result_ret.mat"] 
        if i == "result_ret.mat"
            csvData(fileName + i, 1);
        else
            csvData(fileName + i, 0);
        end
    end
end

%% Exctract no-hands data
disp("No-hands Test");
fileName = "data" + filesep + "no_hands_";
csvData(fileName + "result.mat", 0);

%% Exctract simulation data
disp("Simulation");
fileName = "data" + filesep + "simulation_";
csvSimData(fileName + "result.mat");

%% Clean up
clear fileName i j;
rmpath("functions/");
disp("Done!");