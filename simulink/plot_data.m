clear;
clc;
close all;
load("participant2_noMPC.mat");

%% Convert to timetable
mpc_output = timeseries2timetable(out.SteerTorque);
mpc_statistics = timeseries2timetable(out.MpcStats.execTime, ...
    out.MpcStats.kktValue);
bicycle_state_serial = timeseries2timetable(out.BicycleState.delta, ...
    out.BicycleState.d_delta);
bicycle_state_udp = timeseries2timetable(out.BicycleState.y_P, ...
    out.BicycleState.psi, out.BicycleState.phi, ...
    out.BicycleState.d_phi);
%bicycle_data = timeseries2timetable(out.BicycleData.Switch_State, ...
%    out.BicycleData.Handlebar_Angle, out.BicycleData.Handlebar_Rate, ...
%    out.BicycleData.Gyroscope_X, out.BicycleData.Velocity, ...
%    out.BicycleData.Time_Taken);
%vive_data = timeseries2timetable(out.ViveData.Timestamp, ...
%    out.ViveData.AxisAngleRot_X, out.ViveData.AxisAngleRot_Y, ...
%    out.ViveData.AxisAngleRot_Z, out.ViveData.Pos_X, out.ViveData.Pos_Y,...
%    out.ViveData.Pos_Z, out.ViveData.Rot_W, out.ViveData.Rot_X, ...
%    out.ViveData.Rot_Y, out.ViveData.Rot_Z);
%trial_state = timeseries2timetable(out.TrialState);
%reference = timeseries2timetable(out.Reference);

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
%subplot(1,3,1);
%plot(mpc_output.Time, mpc_output.Data);
%title("mpc torque");
%subplot(1,3,2);
%hold on;
range = timerange(seconds(100),seconds(200));
plot(mpc_statistics.Time(range), ...
    mpc_statistics.execTime(range), ...
    "DisplayName","MPC");
plot(mpc_statistics.Time(range), ...
    1/80*ones(length(mpc_statistics.Time(range)),1), ...
    "DisplayName","Sampling Time");
ylabel("Time taken [s]");
xlabel("Time along the trial [s]");
title("Time taken by the MPC");
legend();
%subplot(1,3,3);
%plot(mpc_statistics.Time, mpc_statistics.kktValue);
%title("kkt value");