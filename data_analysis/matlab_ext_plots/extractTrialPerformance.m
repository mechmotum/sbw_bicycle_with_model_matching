%% README
% This script iterates through participants' data, calculates mean scores and
% score variances for each trial and stores them in a .csv file that can then 
% be used in R to run statistical analysis.
% 
% The results should be saved in "data/" folder.
%
% File containing data from BL and T1 should be called "result_t1.mat"
% File containing data from MTR and T2 should be called "result_t2.mat"
% File containing data from ETR should be called "result_ret.mat"
%
% The script outputs "trials.csv" file into the "performance/" folder.
% Columns: Participant IDs, Group membership, Trial of interest, Mean score,
%    Within-participant score variance
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
variableNames = {'ID', 'Group', 'Trial', 'Score', 'Variance'};
% Participant data
participantIDs = 1:10;
groupNo = ones(10, 1);
groupNo([1,2,4,9,10]) = 2; % Set the group membership
% Performance data
avgArray = zeros(10, 5);
devArray = zeros(10, 5);

%% Read the data and calculate the scores
% Iterate through participants
for j = 1:10 
    disp("Participant " + num2str(j));
    fileName = "data" + filesep + "participant" + num2str(j) + "_";
    
    count = 1;
    % Iterate through trials for BL, MTR and ETR data
    for i = ["result_t1.mat", "result_t2.mat", "result_ret.mat"]
        [avg, dev] = get2minScore(fileName + i);
        avgArray(j, 2*(count-1)+1) = avg(3);
        devArray(j, 2*(count-1)+1) = dev(3);
        count = count + 1;
    end

    % Iterate through trials for T1 and T2 data
    count = 1;
    for i = ["result_t1.mat", "result_t2.mat"]
        [avg, dev] = get6minScore(fileName + i); 
        avgArray(j, 2*count) = avg(7);
        devArray(j, 2*count) = dev(7);
        count = count + 1;
    end
end

%% Write the table
% Convert from wide tables to long table
ID = repmat(participantIDs, 1, 5)';
Group = repmat(groupNo, 5, 1);
Trial = [repmat("BL", 10, 1);
         repmat("T1", 10, 1);
         repmat("MTR", 10, 1);
         repmat("T2", 10, 1);
         repmat("ETR", 10, 1)];
Score = reshape(avgArray, [50,1]);
Variance = reshape(devArray, [50,1]);
trialsTable = table(ID, Group, Trial, Score, Variance, ...
    'VariableNames',variableNames);
% Write the csv file
writetable(trialsTable, "performance" + filesep + "trials.csv");
 
%% Clean up
clear avg dev avgArray devArray;
clear i j count groupNo participantIDs variableNames;
clear fileName Group ID Score Trial Variance;
rmpath("functions/");
disp("Done!");
