function generateOptionsFile()

% User-settable options --- Timing
opts.Ts_mpc = 1/75;         % MPC sample time [s]
opts.Ts_udp = 1/320;        % UDP sample time [s]
opts.Ts_ser = 1/200;        % Serial sample time [s]
opts.Tinterval = 6;         % Time between the gates [s]
opts.TShorizon = 2;         % MPC time horizon [s]
opts.TUhorizon = 5;         % Unity time horizon [s]
opts.Tpreview = 4;          % How soon should MPC see the gate [s]
opts.Tafter = 2;            % How long should MPC remember the gate [s]

% User-settable options --- Gate settings
opts.blockLength = 10;      % Number of gates in one block [-]
opts.gateNumber = 7;        % Number of different gate positions [-]
opts.maxWidth = 0.2;        % The furthest lateral displacement of the gate [m]
                            % (from the centre of the treadmill)
opts.randSeed = 20220729;   % Random seed for the randomized gate positions [-]

% User-settable options --- Bicycle parameters
opts.trackerHeight = 0.9;   % Height of the tracker relative to ground [m]
opts.steerTorqueLim = 3;    % Steering torque limit [Nm]
opts.fCutoff = 8;           % Low-pass filter cutoff frequency [Hz]

% User-settable options --- MPC constraints
opts.ub_x = [0.5;           % lateral position [m],
             inf;           % yaw [rad],
             deg2rad(20);   % lean [rad], 
             deg2rad(40);   % steering angle [rad],
             inf;           % lean rate [rad/s],
             inf];          % steering rate [rad/s]
opts.ub_u = opts.steerTorqueLim; % steering torque [Nm]
opts.lb_x = -opts.ub_x;
opts.lb_u = -opts.ub_u;

% User-settable options --- Cost weights
opts.Q1 = 10;                % Position weight [-]
opts.Q2 = 10;               % Yaw weight [-]
opts.Q3 = 3;                % Lean weight [-]
opts.R = 1;                 % Input weight [-]

% Calculated options
opts.Tend = opts.Tinterval * opts.blockLength; % Length of one block [s]
opts.nSH = opts.TShorizon / opts.Ts_mpc; % Number of steps in the horizon [-]
opts.nUH = opts.TUhorizon / opts.Ts_mpc; % Number of steps in Unity horizon [-]
opts.nX = 6;                % Number of states [-]
opts.nU = 1;                % Number of control inputs [-]

%% Save the options to a file
cd("." + filesep + "mat_files");
save opts.mat opts;
cd("..");

end