%% AAE 590 Lab 1 Report
% Group 1, 9/12/22
% Shawn Prosky, Ellen Nguyen, Lauren Risany, Sidd Subramanyam

clc; clear; close all;
%% constants
L = .1512;                      %rover length
R = .0329;                      %rover wheel radius (not used)
% constants related to each run
v_r = .3;                      %right wheel velocity command [m/s]
v_l = .08;                      %right wheel velocity command [m/s]
t_f = 10;                       %sample time [s]
V = (v_r + v_l) / 2;            %rover velocity [m/s]
w = rad2deg((v_r - v_l) / L);   %rover angular velocity [deg/s]

H = [1, 0, 0; 0, 1, 0];
P_k = eye(3);

%% loading data
measurements = readtable("run_3.csv",'VariableNamingRule','preserve');     %importing csv data
measurements = measurements{6:end,:};               %trimming data to remove leading NaNs
time = measurements(:,1);                           %time vector from 1st col of data [s]
x_meas = measurements(:,2) / 1000;                  %measured x position from data, converting from [mm] to [m]
y_meas = measurements(:,3) / 1000;                  %measured y position from data, converting from [mm] to [m]
dt = mean(diff(time));                              %time step size [s]

%% initializing variables and initial state
predicted = zeros(3, length(time));                 %define a matrix to store predicted [x, y, theta] for each time step
corrected = zeros(3, length(time));                 %define a matrix to store corrected [x, y, theta] for each time step

x_k = x_meas(1);                                    %initial value for x pos [m]
y_k = y_meas(1);                                    %initial value for y pos [m]
theta_k = atan2d((y_meas(2)-y_meas(1)),(x_meas(2)-x_meas(1)));  %initial value heading angle [deg]

%% Setting Q and R Matrices (process and measurement noise covariance)
% Part 2
% Q = [1, 0, 0; 0, 1, 0; 0, 0, 1];
% R = [.1, 0; 0, .1];

% Part 5
Q = [.05, 0, 0; 0, .05, 0; 0, 0, 5];                % Process noise covariance [m], [m], [deg]
R = [.001, 0; 0, .001];                             % measurement noise covariance [m], [m] 

%% compute EKF
for t = 1:length(time)
    
    F = [1, 0, -V*sind(theta_k)*dt; 0, 1, V*cosd(theta_k)*dt; 0, 0, 1];
    
    % predict state
    predicted(:,t) = [V*cosd(theta_k)*dt + x_k; V*sind(theta_k)*dt + y_k; w*dt + theta_k];
    
    % predict covariance
    P_k1 = F*P_k*inv(F) + Q;
   
    % find residual
    resid_k = [x_meas(t); y_meas(t)] - H*predicted(:,t);
   
    % covariance residual
    S = H*P_k1*H' + R;
    
    % Kalman gain
    K = P_k1*H'*inv(S);
    
    % update (x hat)
    corrected(:,t) = predicted(:,t) + K*resid_k;
    P_k1_corrected = (eye(3) - K*H)*P_k1;
    
    % updating the variables for the next loop iteration
    x_k = corrected(1,t);
    y_k = corrected(2,t);
    theta_k = corrected(3,t);
    P_k = P_k1_corrected;
end

%% Extracting variables of interest for ease of analysis
x_pred = predicted(1, :);
y_pred = predicted(2, :);
theta_pred = predicted(3, :);

x_real = corrected(1, :);
y_real = corrected(2, :);
theta_real = corrected(3, :);

%% Plotting
% 2D position plots
axis equal;hold on;
grid on
plot(x_real,y_real,'Linewidth',2);
plot(x_pred, y_pred,'--','Linewidth',2); 
plot(x_meas, y_meas, '--','Linewidth',2)
xlabel('x (m)');
ylabel('y (m)');
legend('Corrected','Predicted','Measured','location','best');
title('2D Position');
set(gca,'FontSize',14)

% Error
figure;
hold on
grid on
plot(time, (x_real-x_pred) * 1000,'LineWidth',2);
plot(time, (y_real-y_pred) * 1000,'LineWidth',2);
xlabel('time (s)');
ylabel('Error (mm)');
legend('X Error','Y Error')
title('Error');
set(gca,'FontSize',14)

% Heading
figure;
hold on;
grid on
plot(time, theta_pred,'LineWidth',2);
plot(time, theta_real, '--','LineWidth',3);
xlabel('Time (s)');
ylabel('Heading Angle (deg)');
legend('Predicted', 'Corrected','location','best','fontSize',14);
title('Group 1 - Heading Angle');
set(gca,'FontSize',14)
% Error Theta NOT applicable 

% MSE
MSE_X = (1 / length(x_real))*sum((x_meas' - x_real).^2) * 1e6; % mm^2
MSE_Y = (1 / length(y_real))*sum((y_meas' - y_real).^2) * 1e6; % mm^2
