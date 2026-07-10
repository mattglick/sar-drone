% Matthew Glick
% 6 DOF

%% Initializations

% States: [px, py, pz, vx, vy, vz, roll, pitch, yaw]

%% Initializations
dt = 0.01;        % time step [s]
t_end = 30;       % simulation time [s]
t = 0:dt:t_end;   % time vector [s]

% 9x9 state matrix
% YOUR CODE — place three 1s in the right positions
A = zeros(9,9);
A(1,4) = 1; % px_dot = vx
A(2,5) = 1; % py_dot = vy
A(3,6) = 1; % pz_dot = vz

% Initial true state [px, py, pz, vx, vy, vz, roll, pitch, yaw]
x0_true = [0;      % px — starting x position [m]
           0;      % py — starting y position [m]
           100;    % pz — altitude [m]
           20;     % vx — forward speed [m/s]
           0;      % vy — no sideslip [m/s]
           0;      % vz — level flight [m/s]
           0;      % roll — wings level [rad]
           0;      % pitch — level [rad]
           0];     % yaw — facing north [rad]

%% Sensor noise
gps_std = 3.0;    % GPS position noise [m]
imu_std = 2;    % IMU velocity noise [m/s]

H = zeros(6,9);

for i = 1:6
    H(i,i) = 1;
end

% Process noise — how much we distrust the model
% 9x9, one entry per state
Q = .01 * eye(9);


% Measurement noise — how much we distrust sensors
% 6x6, GPS is noisier than IMU
R = eye(6);

for i = 1:6
    if i <=3
        R(i,i) = gps_std^2;
    else
        R(i,i) = imu_std^2;
    end
end

% Initial estimate — start with true state plus some uncertainty
x_hat = x0_true;     % [9x1]
P = eye(9);          % [9x9] initial uncertainty

% Storage
x_est = zeros(9, length(t));
x_est(:,1) = x_hat;

%% Calculations

x_true = zeros(9, length(t));
x_true(:,1) = x0_true;

for i = 1:length(t)-1
    
    x_true(:,i+1) = x_true(:,i) +  A * x_true(:,i)* dt;

end

% GPS measures position (states 1,2,3) with noise
z_gps = x_true(1:3,:) + gps_std * randn(3, length(t));

% IMU measures velocity (states 4,5,6) with noise  
z_imu = x_true(4:6,:) + imu_std * randn(3, length(t));

for i = 1:length(t)-1
    % PREDICT
    x_hat_minus = A * x_hat * dt + x_hat;
    P_minus     = A * P * A' + Q;
    
    % UPDATE - combine both sensors into one measurement vector
    z = [z_gps(:,i); z_imu(:,i)];   % 6x1 measurement vector
    K     =( P_minus * H') / (H * P_minus * H' + R); %Kalman Filter Gain K

% if no GPS data, K drops toward zero due to an unreliavle innovation term
% GPS cutting out is "DEAD RECKONING"
    x_hat = x_hat_minus + K * ( z - H * x_hat_minus);
    P     = ( eye(9) - K * H) * P_minus; % P is the Uncertainty value. 
    % P starts at a high uncertainty (identity) and then is recalcuated and
    % should converge over time as the information of the model becomes
    % more sound. 
    %Diagonal entries represent the variance of each state estimate.
    %P reaches a steady value and the uncertainty stops decreasing /
    %changing much.
    %
    x_est(:,i+1) = x_hat;
end

%% Formatting

fprintf('Final x position: %.1f m\n', x_true(1,end));
fprintf('Final altitude: %.1f m\n', x_true(3,end));
fprintf('Final speed: %.1f m/s\n', x_true(4,end));

figure;
plot(t, x_true(1,:), 'b', 'LineWidth', 2);
hold on;
plot(t, z_gps(1,:), 'r.', 'MarkerSize', 3);
ylabel('X position (m)'); xlabel('Time (s)');
legend('True', 'GPS noisy');
title('True vs noisy GPS — x position');

%% Plot results
figure;
subplot(3,1,1);
plot(t, x_true(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, z_gps(1,:), 'r.', 'MarkerSize', 3);
plot(t, x_est(1,:), 'g', 'LineWidth', 2);
legend('True', 'GPS noisy', 'Kalman');
ylabel('X position (m)'); title('6DOF Kalman Filter');

subplot(3,1,2);
plot(t, x_true(3,:), 'b', 'LineWidth', 2); hold on;
plot(t, z_gps(3,:), 'r.', 'MarkerSize', 3);
plot(t, x_est(3,:), 'g', 'LineWidth', 2);
ylabel('Altitude (m)');

subplot(3,1,3);
plot(t, x_true(4,:), 'b', 'LineWidth', 2); hold on;
plot(t, z_imu(1,:), 'r.', 'MarkerSize', 3);
plot(t, x_est(4,:), 'g', 'LineWidth', 2);
ylabel('Vx (m/s)'); xlabel('Time (s)');
legend('True', 'IMU noisy', 'Kalman');

%% Visualizations

%EKF needed in complex maneuvers since the rotation from the body frame to
%the world frame is nonlinear

%EKF computes the Jacobian of the nonlinear dynamics at every timestep, a
%local linear approximation that gets recomputed as the aircraft moves

%agressive maneuvers , the body-to-world rotation becomes nonlinear through
%trigonometric coupling between the attitude and the velocity

%the EKF relinearizes aound the current state at each timestep w/ the use
%of the jacobian. (computationally more expensive)