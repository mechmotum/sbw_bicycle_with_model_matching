%% README
% This script iterates through participants' data, calculates mean scores and
% score variances for each block and stores them in a .csv file that can then 
% be used in R to run statistical analysis.
% 
% The results should be saved in "data/" folder.
%
% File containing data from BL and T1 should be called "result_t1.mat"
% File containing data from MTR and T2 should be called "result_t2.mat"
% File containing data from ETR should be called "result_ret.mat"
%
% The script outputs two csv files "avg_blocsk.csv" and "dev_blocks.csv" into
% the "performance/" folder.
% Columns: Participant IDs, Group membership, Column for each block
%
% This script uses two helper functions "get2minScore.m" and "get6minScore.m", 
% which need to be located in the "functions/" folder.
% 
% Written by S. Drauksas, 2022

clear;
clc;
addpath("functions/");

%% Set up required arrays
% CSV header
variableNames = {'ID', 'Grp', ...
    'BL.1', 'BL.2', ...
    'T1.1', 'T1.2', 'T1.3', 'T1.4', 'T1.5', 'T1.6', ...
    'MTR.1', 'MTR.2', ...
    'T2.1', 'T2.2', 'T2.3', 'T2.4', 'T2.5', 'T2.6', ...
    'ETR.1', 'ETR.2'};
% Participant data
participantIDs = 1:10;
groupNo = ones(10, 1);
groupNo([1,2,4,9,10]) = 2; % Set the group membership
% Performance data
avgArray = zeros(10, 18);
devArray = zeros(10, 18);

%% Read the data and calculate the scores
% Iterate through participants
for j = 1:10 
    disp("Participant " + num2str(j));
    fileName = "data" + filesep + "participant" + num2str(j) + "_";
    
    count = 1;
    % Iterate through trials for BL, MTR and ETR data
    for i = ["result_t1.mat", "result_t2.mat", "result_ret.mat"]
        [avg, dev] = get2minScore(fileName + i);
        avgArray(j, 8*(count-1)+1) = avg(1);
        avgArray(j, 8*(count-1)+2) = avg(2);
        devArray(j, 8*(count-1)+1) = dev(1);
        devArray(j, 8*(count-1)+2) = dev(2);
        count = count + 1;
    end

    % Iterate through trials for T1 and T2 data
    count = 1;
    for i = ["result_t1.mat", "result_t2.mat"]
        [avg, dev] = get6minScore(fileName + i); 
        avgArray(j, 8*(count-1)+3 : 8*(count-1)+8) = avg(1:6);
        devArray(j, 8*(count-1)+3 : 8*(count-1)+8) = dev(1:6);
        count = count + 1;
    end
end

%% Write the table
% Mean scores
avgTable = table(participantIDs', groupNo, ...
    avgArray(:,1), avgArray(:,2),...
    avgArray(:,3), avgArray(:,4), avgArray(:,5), avgArray(:,6), avgArray(:,7), avgArray(:,8),...
    avgArray(:,9), avgArray(:,10),...
    avgArray(:,11), avgArray(:,12), avgArray(:,13), avgArray(:,14), avgArray(:,15), avgArray(:,16), ...
    avgArray(:,17), avgArray(:,18), ...
    'VariableNames',variableNames);
% Score variances
devTable = table(participantIDs', groupNo, ...
    devArray(:,1), devArray(:,2),...
    devArray(:,3), devArray(:,4), devArray(:,5), devArray(:,6), devArray(:,7), devArray(:,8),...
    devArray(:,9), devArray(:,10),...
    devArray(:,11), devArray(:,12), devArray(:,13), devArray(:,14), devArray(:,15), devArray(:,16), ...
    devArray(:,17), devArray(:,18), ...
    'VariableNames',variableNames);
% Write the csv files
writetable(avgTable, "performance" + filesep + "avg_blocks.csv");
writetable(devTable, "performance" + filesep + "dev_blocks.csv");
 
%% Clean up
clear avg dev avgArray devArray;
clear i j count groupNo participantIDs variableNames;
clear fileName;
rmpath("functions/");
disp("Done!");
