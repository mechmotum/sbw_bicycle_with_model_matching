function [avg, dev] = get6minScore(filename)
% Calculates and returns mean scores and within-participant score variance of
% the T1 and T2 trials.
%
% INPUTS:
%   filename - string with a filename and extension of the file of interest
% OUTPUTS:
%   avg(1) -> avg(6) - average scores of blocks 1 -> 6
%   avg(7) - average score of all stars
%   dev(1) -> dev(6) - standard deviation of blocks 1 -> 6
%   dev(7) - standard deviation of all stars
%
% Written by S. Drauksas, 2022

%% Load the file ant extract the data
load(filename);
% Extract bicycle and target locations
bikePos = timeseries2timetable(out.BicycleState.y_P);
targetPos = timeseries2timetable(out.Reference);
% Extract the researcher's button data
button = timeseries2timetable(out.TrialState);
% Synchronise different sample rates
pos = synchronize(targetPos, bikePos, button);

%% Find the timerange of the trial
temp = find(button.Data == 1); % When was the button pressed?
protocolStart = temp(1);

t = button.Time(protocolStart) + seconds(126) : ... % First target is 126s after start
    seconds(6) : ... % Other targets come every 6 seconds
    button.Time(protocolStart) + seconds(480); % 6 min trial

%% Calculate scores
% Find distances, store into 1st column
scores(:,1) = abs(pos.y_P(t) - pos.Data_targetPos(t, 1)); 
% Find scores, store into 2nd column
for i=1:length(scores)
    if scores(i,1) <= 0.02
        scores(i,2) = 100;
    elseif scores(i,1) <= 0.22
        scores(i,2) = 500 * (0.22 - scores(i,1));
    else
        scores(i,2) = 0;
    end
end

%% Store into the output arrays
avg(1) = mean(scores(1:10, 2)); % 1st block
avg(2) = mean(scores(11:20, 2)); % 2nd block
avg(3) = mean(scores(21:30, 2)); % 3rd block
avg(4) = mean(scores(31:40, 2)); % 4th block
avg(5) = mean(scores(41:50, 2)); % 5th block
avg(6) = mean(scores(51:60, 2)); % 6th block
avg(7) = mean(scores(:,2)); % All blocks

dev(1) = std(scores(1:10, 2)); % 1st block
dev(2) = std(scores(11:20, 2)); % 2nd block
dev(3) = std(scores(21:30, 2)); % 3rd block    
dev(4) = std(scores(31:40, 2)); % 4th block    
dev(5) = std(scores(41:50, 2)); % 5th block
dev(6) = std(scores(51:60, 2)); % 6th block
dev(7) = std(scores(:,2)); % All blocks

disp("=====6 MIN TRIAL=====");
disp("Average  all: " + num2str(avg(7)));
disp("Variance all: " + num2str(dev(7)));
disp("=====================");

end