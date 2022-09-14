function [avg, dev] = get2minScore(filename)
% Calculates and returns mean scores and within-participant score variance of
% the BL, MTR and ETR trials.
%
% INPUTS:
%   filename - string with a filename and extension of the file of interest
% OUTPUTS:
%   avg(1) - average score of block 1
%   avg(2) - average score of block 2
%   avg(3) - average score all stars
%   dev(1) - standard deviation of block 1
%   dev(2) - standard deviation of block 2
%   dev(3) - standard deviation of all stars
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

t = button.Time(protocolStart) + seconds(6) : ... % First target is 6s after start
    seconds(6) : ... % Other targets come every 6 seconds
    button.Time(protocolStart) + seconds(120); % 2 min trial

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
avg(3) = mean(scores(:,2)); % All blocks

dev(1) = std(scores(1:10, 2)); % 1st block
dev(2) = std(scores(11:20, 2)); % 2nd block
dev(3) = std(scores(:,2)); % All blocks

disp("=====2 MIN TRIAL=====");
disp("Average  all: " + num2str(avg(3)));
disp("Variance all: " + num2str(dev(3)));
disp("=====================");

end