function [scores] = get6minStars(filename)
% Calculates and returns scores of the BL, MTR and ETR trials.
%
% INPUTS:
%   filename - string with a filename and extension of the file of interest
% OUTPUTS:
%   scores - an array of 60 star scores
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

t = button.Time(protocolStart) + seconds(126) : ... % First target is 6s after start
    seconds(6) : ... % Other targets come every 6 seconds
    button.Time(protocolStart) + seconds(480); % 6 min trial

%% Calculate scores
% Find distances
displace = abs(pos.y_P(t) - pos.Data_targetPos(t, 1)); 
% Find scores
for i=1:length(displace)
    if displace(i) <= 0.02
        scores(i) = 100;
    elseif displace(i) <= 0.22
        scores(i) = 500 * (0.22 - displace(i));
    else
        scores(i) = 0;
    end
end

end