clc
clear all


for i=1:30  


%% Set Simulation Constants
clearvars -except i

% Gyro Constants
w_bias = 0*[0.5 0.2 0.15];     % rad/s
gyro_noise = (0.35*pi/180)^2;  % rad^2/s^2
% gyro_noise = 0;          % rad^2/s^2

% Magnetometer Constants
B_bias = [0 0 0];               % Tesla
B_noise = ((1*0.0052)/1E5)^2;                % Tesla^2

% Randomized
rand_w_x=rand*.1;
rand_w_y=rand*.1;
rand_w_z=rand*.1;
w_init_s = [rand_w_x rand_w_y rand_w_z];	% rad/s
init_attitude=[1 0 0 0]+rand(1,4)*.5-.25;  % changed to reduce the initial error
init_attitude=init_attitude/norm(init_attitude);
q_i_s0 = init_attitude;
[longstr1,longstr2, rand_TLE]=GetTLE(0);

% Constant
%w_init_s = [.046 .17 .039];	% rad/s
%q_i_s0 = [1 0 0 0];
%[longstr1,longstr2, rand_TLE]=GetTLE(3);

% Operating Enviroment
Mean_Velocity=norm(w_init_s);
Time_Start = 0;          % min after epoch
run_time = 50000;                   % sec
Ts = .1;                           % sec

% Initial 
q_s_c = [1 0 0 0];
Torque_s = [0 0 0];
I_c = [3.4 3.4 1.9];

A=mat2str(w_init_s,2);
B=num2str(Mean_Velocity);
C=int2str(rand_TLE);
D=mat2str(init_attitude,2);
E=int2str(i);

fprintf('Initializing run %s.\n Initial angular rates: %s. Average: %s.\n Initial attitude: %s.\n TLE: %s.\n\n',E,A,B,D,C); 



% Initialize the SGP4 propagator
SGP4_Setup(longstr1, longstr2)
ECEF_Init = sgp4(Time_Start)*1000;
LLA_Init = ecef2lla(ECEF_Init');
Lat = LLA_Init(1);
Long = LLA_Init(2);
Alt = LLA_Init(3)/1000;
B_Init_Inertial = wrldmagm(Alt, Lat, Long, 2013)*1E-9;   % Convert nT to T;
NED_Init = dcmecef2ned(Lat, Long)*ECEF_Init;

Decimation = 1;

%% Run The Simulation;
sim('UKF_10hz');

%% Plot the Results
Simulation_Time = Time_Start:(Decimation*Ts):(Time_Start+run_time);

% Line_colors = [1 0 0; 0 0.5 0; 0 0 1; 0.7 0 0.7];
% Line_styles = {'-', '--', ':'};
% Line_width = [1, 2];
% 
% % Plot Bias
% figure();
% hold('on')
% grid('on');
% for idx = 1:3;
% plot(Simulation_Time, UKF_Gyro_Bias(:,idx)*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:));
% end
% xlabel('Time (seconds)');
% ylabel('Gyro Bias (deg/s)');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% 
% % Plot Angular rate
% figure();
% hold('on');
% grid('on');
% for idx = 1:3
% plot(Simulation_Time, UKF_Angular_Velocity(:,idx)*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% plot(Simulation_Time, Actual_Angular_Velocity(:,idx)*180/pi,...
%     'LineStyle', Line_styles{2}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(2));
% end
% xlabel('Time (seconds)');
% ylabel('Angular Velocity (deg/s)');
% 
% % Fake legend
% ax = axes; hold('on');
% plot(rand(1,2), 'color', [1 0 0], 'visible', 'off');
% plot(rand(1,2), 'color', [0 0.5 0], 'visible', 'off');
% plot(rand(1,2), 'color', [0 0 1], 'visible', 'off');
% set(ax, 'visible', 'off');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% 
% % Plot Angular Velocity Error
% figure();
% hold('on');
% grid('on');
% for idx = 1:3
% plot(Simulation_Time, (Actual_Angular_Velocity(:,idx) - UKF_Angular_Velocity(:,idx))*180/pi,...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% end
% xlabel('Time (seconds)');
% ylabel('Angular Velocity Error (deg/s)');
% legend('X', 'Y', 'Z');
% PrettyUpPlot
% ylim([-3 3])
% %% Plot Attitude
% figure();
% hold('on');
% grid('on');
% for idx = 1:4;
% plot(Simulation_Time, UKF_Attitude(:,idx),...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(1));
% plot(Simulation_Time, Actual_Attitude(:,idx),...
%     'LineStyle', Line_styles{2}, 'Color', Line_colors(idx,:),...
%     'LineWidth', Line_width(2));
% end
% xlabel('Time (seconds)');
% xlabel('Quaternion Attitude');
% PrettyUpPlot
% 
% % Plot Attitude Error
% figure();
% hold('on');
% grid('on');
% for idx = 1:3;
% plot(Simulation_Time, Attitude_Error(:,idx),...
%     'LineStyle', Line_styles{1}, 'Color', Line_colors(idx,:));
% end
% xlabel('Time (seconds)');
% ylabel('Quaternion Attitude Error');
% legend('Z', 'Y', 'X');
% PrettyUpPlot

% Plot Attitude Error
h=figure();
hold('on');
grid('on');
% error_mag = sqrt(sum(Attitude_Error.^2,2));
error_mag=Attitude_Error;
plot(Simulation_Time, error_mag, 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Euler Angle Error Magnitude (degrees)');
legend('Error Magnitude');
PrettyUpPlot

saveas(h,sprintf('w %s, Mean Velocity %s, TLE %s Attitude Guess %s.png',A,B,C,D));
close(h);

end