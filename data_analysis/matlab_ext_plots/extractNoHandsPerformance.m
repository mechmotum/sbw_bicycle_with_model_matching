%% README
% This script calculates scores of the no-hands trial and stores them in a .csv 
% file that can then be used in R to run statistical analysis.
% 
% The results should be saved in "data/" folder.
%
% File containing no-hands data should be called "no_hands_result.mat"
% File containing ETR trial data of the participants should be called 
% "result_ret.mat"
%
% The script outputs "no_hands.csv" file into the "performance/" folder.
% Columns: Scenario, Scores
%
% This script uses two helper functions "get2minStars.m" and "get6minStars.m", 
% which need to be located in the "functions/" folder.
% 
% Written by S. Drauksas, 2022

clear;
clc;
addpath("functions/");

%% Set up required arrays
variableNames = {'scenario', 'score'};

%% Read the data and calculate the scores
disp("No hands");
% Load the corresponding folder into PATH
fileName = "data" + filesep + "no_hands_result.mat";

% Get Baseline scores
scoreNC = get2minStars(fileName);

% Get Controller scores
scoreC = get6minStars(fileName); 

% Get participants' ETR scores
% Iterate through participants
for j = 1:10 
    disp("Participant " + num2str(j));
    fileName = "data" + filesep + "participant" + num2str(j) + "_result_ret.mat";
    scoreETR(20*(j-1)+1 : 20*j) = get2minStars(fileName);
end

%% Write the table
% Convert from wide tables to long table
Scenario = [repmat("NH_NC", 20, 1);
            repmat("NH_C", 60, 1);
            repmat("ETR", 200, 1)];
Score = [scoreNC';
         scoreC';
         scoreETR'];
noHandsTable = table(Scenario, Score, ...
    'VariableNames',variableNames);
% Write the csv file
writetable(noHandsTable, "performance" + filesep + "no_hands.csv");

%% Clean up
clear j variableNames fileName;
clear scoreNC scoreC scoreETR Scenario Score;
rmpath("functions/");
disp("Done!");
