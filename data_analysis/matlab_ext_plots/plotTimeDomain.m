%% README
% This script plots the time domain graphs seen in the thesis.
% 
% The script needs "no_hands_result.mat" and "simulation_result.mat" data files
% that should be saved in "data/" folder.
%
% The script outputs three graphs "SimSmall.png", "SimRates.png" and 
% "NHPosition.png" into the "figures/" folder.
% 
% Written by S. Drauksas, 2022

clc;
clear;

%% Extract data
% Load the no-hands trial data
load("data" + filesep + "no_hands_result.mat");
% Extract bicycle state
latPos = timeseries2timetable(out.BicycleState.y_P);
steerAngle = timeseries2timetable(out.BicycleState.delta);
rollAngle = timeseries2timetable(out.BicycleState.phi);
yawAngle = timeseries2timetable(out.BicycleState.psi);
steerRate = timeseries2timetable(out.BicycleState.d_delta);
rollRate = timeseries2timetable(out.BicycleState.d_phi);
% Extract MPC data
mpcTorque = timeseries2timetable(out.SteerTorque);
% Extract reference data, keep only the current timestep values, discard future
ref = timeseries2timetable(out.Reference);
ref.Data = ref.Data(:,1);
% Extract data from the researcher's button
button = timeseries2timetable(out.TrialState);
% Synchronise data between different sample rates
synchNH = synchronize(latPos, ...
                      steerAngle, ...
                      rollAngle, ...
                      yawAngle, ...
                      steerRate, ...
                      rollRate, ...
                      mpcTorque, ...
                      ref, ...
                      button);

% Load the simulation data
load("data" + filesep + "simulation_result.mat");
% Extract bicycle state
bState = timeseries2timetable(out.BicycleState);
% Extract MPC data
mpcTorque = timeseries2timetable(out.SteerTorque);
% Put data into one timetable
synchSim = synchronize(bState, mpcTorque);

%% Find the time range of the trial
% No-hands trial
temp = find(button.Data == 1); % When was the button pressed?
protocolStartNH = temp(1);
% No-hands with controller trial timerange and the target reference
tNH = timerange(button.Time(protocolStartNH) + seconds(120), ...
                button.Time(protocolStartNH) + seconds(480));
tNH_ref = button.Time(protocolStartNH) + seconds(126):...
          seconds(6):...
          button.Time(protocolStartNH) + seconds(480);

% Simulation
tSim = timerange(seconds(120), ...
                 seconds(480));

%% Plot first figure
f1 = figure(Units="centimeters", Position=[0 0 17.8 13]);
% Lateral Position
ax1 = subplot(5,1,1);
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.y_P(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_bState(tSim,1), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
plot(synchNH.Time(tNH_ref) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.Data_ref(tNH_ref), ...
     'o', LineWidth=1.5, Color="#E03C31"); % Targets
ylim([-0.3; 0.3]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[m]");
subtitle("Lateral Position");
ax1.YGrid = true;

% Yaw Angle
ax2 = subplot(5,1,2);
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.psi(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_bState(tSim,2), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.1; 0.1]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[rad]");
subtitle("Yaw Angle");
ax2.YGrid = true;

% Roll Angle
ax3 = subplot(5,1,3);
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.phi(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_bState(tSim,3), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.05; 0.05]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[rad]");
subtitle("Roll Angle");
ax3.YGrid = true;

% Steering Angle
ax4 = subplot(5,1,4);
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.delta(tNH) - deg2rad(0.17), ... % Get rid of angle offset
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_bState(tSim,4), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.05; 0.05]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[rad]");
subtitle("Steer Angle");
ax4.YGrid = true;

% MPC Torque
ax5 = subplot(5,1,5);
colororder({'#00A6D6', '#6CC24A'});
yyaxis left
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.Data_mpcTorque(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
ylim([-1; 1]);
ylabel("[Nm]");
yyaxis right
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_mpcTorque(tSim), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.1; 0.1]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
xlabel("Time [s]");
ylabel("[Nm]");
subtitle("MPC Torque");
ax5.YGrid = true;

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
exportgraphics(f1, "figures/SimSmall.png", Resolution=320) % Export the figure

%% Plot second figure
f2 = figure(Units="centimeters", Position=[0 0 17.8 9.0]);
% Roll rate
ax1 = subplot(2,1,1);
colororder({'#00A6D6', '#6CC24A'});
yyaxis left
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.d_phi(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
ylim([-0.1; 0.1]);
ylabel("[rad/s]");
yyaxis right
plot(synchSim.Time(tSim) - seconds(120), ...
     synchSim.Data_bState(tSim,5), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.05; 0.05]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[rad/s]");
subtitle("Roll Rate");
ax1.YGrid = true;

% Steering rate
ax2 = subplot(2,1,2);
colororder({'#00A6D6', '#6CC24A'});
yyaxis left
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.d_delta(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
ylim([-0.3; 0.3]);
ylabel("[rad/s]");
yyaxis right
plot(synchSim.Time(tSim) - seconds(120),...
     synchSim.Data_bState(tSim,6), ...
     LineWidth=1.5, LineStyle="--", Color="#6CC24A"); % Simulation
ylim([-0.05; 0.05]);
xlim([seconds(328), seconds(338)]); % 10 second excerpt
ylabel("[rad/s]");
subtitle("Steer Rate");
ax2.YGrid = true;

linkaxes([ax1, ax2], 'x')
exportgraphics(f2, "figures/SimRates.png", Resolution=320) % Export the figure

%% Plot third figure
% Get the timerange for the 2 min no controller trial
tNC = timerange(button.Time(protocolStartNH), ...
                button.Time(protocolStartNH) + seconds(120));
tNC_ref = button.Time(protocolStartNH) + seconds(6):...
          seconds(6):...
          button.Time(protocolStartNH) + seconds(120);
% Create figure
f3 = figure(Units="centimeters", Position=[40 40 17.8 9]);
% No controller
ax1 = subplot(2,1,1);
plot(synchNH.Time(tNC) - button.Time(protocolStartNH), ...
     synchNH.y_P(tNC), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchNH.Time(tNC_ref) - button.Time(protocolStartNH), ...
     synchNH.Data_ref(tNC_ref), ...
     'o', LineWidth=1.5, Color="#E03C31"); % Targets
ylim([-0.5; 0.5]);
xlim([seconds(60), seconds(120)]); % 60 second excerpt
ylabel("Position [m]");
subtitle("Baseline");
ax1.YGrid = true;

% Controller
ax2 = subplot(2,1,2);
plot(synchNH.Time(tNH) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.y_P(tNH), ...
     LineWidth=1.5, Color="#00A6D6"); % No-hands
hold on;
plot(synchNH.Time(tNH_ref) - button.Time(protocolStartNH) - seconds(120), ...
     synchNH.Data_ref(tNH_ref), ...
     'o', LineWidth=1.5, Color="#E03C31"); % Targets
ylim([-0.5; 0.5]);
xlim([seconds(300), seconds(360)]); % 60 second excerpt
xlabel("Time [s]");
ylabel("Position [m]");
subtitle("Controller")
ax2.YGrid = true;

exportgraphics(f3, "figures/NHPosition.png", Resolution=320) % Export the figure

%% Clean up
clear ax1 ax2 ax3 ax4 ax5;
clear bState button latPos mpcTorque ref rollAngle rollRate steerAngle;
clear steerRate synchNH synchSim yawAngle out protocolStartNH temp;
clear tNC tNC_ref tNH tNH_ref tSim;
disp("Done!");