%clear;
%clc;
%close all;
%load("simonas.mat");

%% Convert to timetable
mpc_output = timeseries2timetable(out.SteerTorque);
mpc_statistics = timeseries2timetable(out.MpcStats.execTime, ...
    out.MpcStats.kktValue);
bicycle_state_serial = timeseries2timetable(out.BicycleState.delta, ...
    out.BicycleState.d_delta);
bicycle_state_udp = timeseries2timetable(out.BicycleState.y_P, ...
    out.BicycleState.psi, out.BicycleState.phi, ...
    out.BicycleState.d_phi);
bicycle_data = timeseries2timetable(out.BicycleData.Switch_State, ...
    out.BicycleData.Handlebar_Angle, out.BicycleData.Handlebar_Rate, ...
    out.BicycleData.Gyroscope_X, out.BicycleData.Velocity, ...
    out.BicycleData.Time_Taken);
vive_data = timeseries2timetable(out.ViveData.Timestamp, ...
    out.ViveData.AxisAngleRot_X, out.ViveData.AxisAngleRot_Y, ...
    out.ViveData.AxisAngleRot_Z, out.ViveData.Pos_X, out.ViveData.Pos_Y,...
    out.ViveData.Pos_Z, out.ViveData.Rot_W, out.ViveData.Rot_X, ...
    out.ViveData.Rot_Y, out.ViveData.Rot_Z);
trial_state = timeseries2timetable(out.TrialState);
reference = timeseries2timetable(out.Reference);

bicycle_state = synchronize(bicycle_state_udp, bicycle_state_serial);

%% Plotting
figure;
hold on;
subplot(2,3,1);
plot(bicycle_state.Time, bicycle_state.y_P);
title("lateral pos");
subplot(2,3,2);
plot(bicycle_state.Time, bicycle_state.psi);
title("yaw");
subplot(2,3,3);
plot(bicycle_state.Time, bicycle_state.delta);
title("steering angle");
subplot(2,3,4);
plot(bicycle_state.Time, bicycle_state.phi);
title("lean angle");
subplot(2,3,5);
plot(bicycle_state.Time, bicycle_state.d_delta);
ylim([-pi pi]);
title("steering rate");
subplot(2,3,6);
plot(bicycle_state.Time, bicycle_state.d_phi);
title("lean rate");

figure;
hold on;
subplot(1,3,1);
plot(mpc_output.Time, mpc_output.Data);
title("mpc torque");
subplot(1,3,2);
hold on;
plot(mpc_statistics.Time, mpc_statistics.execTime);
plot(mpc_statistics.Time, 1/80*ones(length(mpc_statistics.Time),1));
title("mpc time");
subplot(1,3,3);
plot(mpc_statistics.Time, mpc_statistics.kktValue);
title("kkt value");

figure;
hold on;
plot(bicycle_data.Time, bicycle_data.Switch_State);