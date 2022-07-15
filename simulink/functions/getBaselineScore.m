function getBaselineScore(filename)

load(filename);

bikePos = timeseries2timetable(out.BicycleState.y_P);
gatePos = timeseries2timetable(out.Reference);
button = timeseries2timetable(out.TrialState);
pos = synchronize(gatePos, bikePos, button);

temp = find(button.Data == 1);
protocolStart = temp(1);
%temp = find(button.Data == 0);
%protocolEnd = temp(protocolStart);

t = button.Time(protocolStart) + seconds(6) : ...
        seconds(6) : ...
            button.Time(protocolStart) + seconds(120);

scores(:,1) = abs(pos.y_P(t) - pos.Data_gatePos(t, 1)); 

for i=1:length(scores)
    if scores(i,1) <= 0.02
        scores(i,2) = 100;
    elseif scores(i,1) <= 0.22
        scores(i,2) = 500 * (0.22 - scores(i,1));
    else
        scores(i,2) = 0;
    end
end

avg1 = mean(scores(1:10, 2));
avg2 = mean(scores(11:20, 2));
avg = mean(scores(:,2));

disp("Average 1st Block: " + num2str(avg1));
disp("Average 2nd Block: " + num2str(avg2));
disp("Average of both: " + num2str(avg));

end