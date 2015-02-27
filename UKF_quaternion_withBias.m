function [Attitude_sensor, w_sensor, w_bias_sensor]  = UKF_quaternion_withBias(B_ref, B_sat, w_gyro, Torque_s, Ts)
%#codegen
% UKF_quaternion
% Author: Brandon Jackson
% Contact: bajackso@mtu.edu
% Date: 22 May 2013
%
% This function performs an Unscented Kalman Filter to determine the
% spacecraft attitude.
%
% The state vector is: [Error Quaternion (3,1); Angular_Velocity; Gyro Bias (3,1)]

% Vectors will be normalized, direction not magnitude, is what is important
% values closer to one will have less issues with roundoff.
B_sat = B_sat./sqrt(sum(B_sat.^2));
B_ref = B_ref./sqrt(sum(B_ref.^2));

% Declare persistent variables. These variables behave similiar to global
% variables although they are only present within this function
persistent P_km1km1 x_km1km1 x_kk R_k Q_k Lambda alpha K Beta q_s_c I_c

%% Initialize filter if necessary
if isempty(x_km1km1)
    q_s_c = [1; 0; 0; 0];
    I_c = [3.4 3.4 1.9];

    % Initialize the initial error covariance matrix
    P_km1km1 = diag([1 1 1 0.1 0.1 0.1 1 1 1])*1E-3;
    P_kk = P_km1km1;
    
    % Initialize the state vector
    x_km1km1 = zeros(10,1);
    x_km1km1(1:4,1) = [1 0 0 0];
    x_km1km1(5:7,1) = RotateVecSensor2Cont(w_gyro, q_s_c);
    x_km1km1(8:10,1) = [0 0 0]; % Gyro Bias
    x_kk = x_km1km1;
    
    % Definitions
    L = 9; % number of states (3 quaternions & 3 angular rates)
    alpha = sqrt(3);
    K = 0;
    Lambda = alpha^2*(L+K) - L;
    R_k = diag([0.05 0.05 0.05 0.012 0.012 0.012])*1E-3;  % Measurement Noise Covariance
    Q_k = diag([1 1 1 2 2 2 0.01 0.01 0.01])*1E-6;   % Matrix for Process noise covariance
    Beta = 2;
else
    % already initiated, go to regular operating mode
    L = 9;
    % Allocate Returned Variable Sizes
    x_kk = zeros(10,1); %#ok
    Attitude_sensor = zeros(4,1); %#ok
    w_bias = zeros(3,1); %#ok
    
    %% Sigma Points
    % Calculate the error sigma points
    DX_sigma = chol( (L+Lambda)*P_km1km1 );
    dX_km1km1 = [zeros(L,1) -DX_sigma DX_sigma];
    
    % Calculate the full sigma points
    X_km1km1_temp = zeros(4,2*L+1); %#ok
    X_km1km1_temp = [sqrt(1-sum(dX_km1km1(1:3,:).^2,1)); dX_km1km1(1:3,:)];
    X_km1km1 = [quatmultiply(X_km1km1_temp, x_km1km1(1:4,1));...	% Quaternion
        x_km1km1(5:7,1)*ones(1,2*L+1) + dX_km1km1(4:6,:);...                % Angular Velocity
        x_km1km1(8:10,1)*ones(1,2*L+1) + dX_km1km1(7:9,:)];                 % Gyro Bias
    
    % Propogate the sigma points with the nonlinear system model and the input
    % vector:
    Torque_c = RotateVecSensor2Cont(Torque_s, q_s_c);
    X_kkm1 = RK4(X_km1km1, I_c, Torque_c, Ts);
    
    % Calculate the priori state estimate
    W0_m = Lambda/(L + Lambda);
    W0_c = Lambda/(L + Lambda) + (1-alpha^2 + Beta);
    Wi_cm = 1/(2*(L+Lambda));
    x_kkm1 = sum([W0_m*X_kkm1(1:10,1),Wi_cm*X_kkm1(1:10,2:2*L+1)],2); % sum along row
    x_kkm1(1:4,1) = x_kkm1(1:4,1)./sqrt(sum(x_kkm1(1:4,1).^2));
    
    
    % Calculate full error state
    dX_kkm1_temp = zeros(4, 2*L+1); %#ok
    dX_kkm1_temp = quatmultiply(X_kkm1(1:4,:), quatinv(x_kkm1(1:4,1)));
    dX_kkm1 = [dX_kkm1_temp(2:4,1:2*L+1);...
        X_kkm1(5:10,:) - x_kkm1(5:10,1)*ones(1,2*L+1)];
    
    % Calculate mean error state
    dx_kkm1 = sum([W0_m*dX_kkm1(:,1),Wi_cm*dX_kkm1(:,2:2*L+1)],2); % sum along row
    
    % Calculate the priori error covariance matrix
    P_kkm1 = zeros(L,L);
    for i = 1:2*L+1
        if i == 1;
            P_kkm1 = W0_c*(dX_kkm1(:,i) - dx_kkm1)*...
                (dX_kkm1(:,i) - dx_kkm1)' + P_kkm1;
        else
            P_kkm1 = Wi_cm*(dX_kkm1(:,i) - dx_kkm1)*...
                (dX_kkm1(:,i) - dx_kkm1)' + P_kkm1;
        end
    end
    P_kkm1 = P_kkm1 + Q_k;
    
    % Propogate the sigma points through the sensor model in order to obtain
    % the transformed sigma points. Predicted measurements must be in the
    % satelite reference frame.
    Z_kkm1_control = zeros(6, 2*L+1); %#ok
    z_kkm1_control = zeros(6,1); %#ok
    Z_kkm1_control = SensorModel(X_kkm1, B_ref);
    z_kkm1_control = sum([W0_m*Z_kkm1_control(:,1),Wi_cm*Z_kkm1_control(:,2:end)],2); % sum along row
    
    % Calculate the posteriori state estimate using the measurement vector
    % containing measurements obtained at k:
    P_zkzk = zeros(6,6);
    P_xkzk = zeros(9,6);
    for i = 1:2*L+1
        if i == 1;
            P_zkzk = W0_c*(Z_kkm1_control(:,i) - z_kkm1_control)*...
                (Z_kkm1_control(:,i) - z_kkm1_control)' + P_zkzk;
            P_xkzk = W0_c*(dX_kkm1(:,i) - dx_kkm1)*...
                (Z_kkm1_control(:,i) - z_kkm1_control)' + P_xkzk;
        else
            P_zkzk = Wi_cm*(Z_kkm1_control(:,i) - z_kkm1_control)*...
                (Z_kkm1_control(:,i) - z_kkm1_control)' + P_zkzk;
            P_xkzk = Wi_cm*(dX_kkm1(:,i) - dx_kkm1)*...
                (Z_kkm1_control(:,i) - z_kkm1_control)' + P_xkzk;
        end
    end
    P_zkzk = P_zkzk + R_k;
    
    % Calculate the Kalman gain
    K_k = P_xkzk/P_zkzk;
    
    % Normalize predicted maganetic field vector
    z_kkm1_control(1:3) = z_kkm1_control(1:3)./sqrt(sum(z_kkm1_control(1:3).^2)); % normalize Z vector
    
    % Rotate measurements from sensor to control reference frame.
    B_control = RotateVecSensor2Cont(B_sat, q_s_c);
    w_control = RotateVecSensor2Cont(w_gyro, q_s_c);
    dx_kk = K_k*([B_control; w_control] - z_kkm1_control);
    
    % Calculate the full state
    x_kk = [quatmultiply([sqrt(1-sum(dx_kk(1:3).^2)); dx_kk(1:3)], x_kkm1(1:4,1));...
        x_kkm1(5:7,1) + dx_kk(4:6,1);...
        x_kkm1(8:10,1) + dx_kk(7:9,1)];
    
    % Calculate the a posteriori error covariance
    P_kk = P_kkm1 - K_k*P_zkzk*K_k';
end

%% Store, Rotate, and Deliver
% Store states for next call
P_km1km1 = P_kk;
x_km1km1 = x_kk;

% Convert states from satellite controller frame to the reference frame
Attitude_sensor = quatmultiply(x_kk(1:4,1), quatinv(q_s_c));
w_sensor = RotateVecCont2Sensor(x_kk(5:7,1), q_s_c);
w_bias_sensor = RotateVecCont2Sensor(x_kk(8:10,1), q_s_c);

end

function X_kkm1 = RK4(X_km1km1, I_c, Torque_c, Ts)
% This function performs a fourth-order Runge-Kutta integration on the
% kinematic equation for the satellite to propogate the sigma points
% through it's current dynamic behavior to develop a prediction state
%
% X_km1km1 is the quaternion sigma points
% w is the angular velocities
% Ts is the descrete sample time

X_kkm1 = zeros(size(X_km1km1));
for idx = 1:size(X_km1km1,2)

    k1 = Kinematics(X_km1km1(1:10,idx), I_c, Torque_c);
    k2 = Kinematics(X_km1km1(1:10,idx) + 0.5*k1*Ts, I_c, Torque_c);
    k3 = Kinematics(X_km1km1(1:10,idx) + 0.5*k2*Ts, I_c, Torque_c);
    k4 = Kinematics(X_km1km1(1:10,idx) + k3*Ts, I_c, Torque_c);
    
    X_kkm1(1:10,idx) = X_km1km1(1:10,idx) + (k1 + 2*k2 + 2*k3 + k4)*Ts/6;
    

end % End of for
end

function Z_kkm1 = SensorModel(X_kkm1, B_ref)
% This function propagates the sigma points through the sensor model to
% obtain the transformed sigma points (Predicted Measurements).
%
% x_kkm1 is the propagated sigma points
% B_ref is the reference frame magnetic field vector

Z_kkm1 = zeros(6,19);

for idx = 1:size(X_kkm1,2)
  
    % Magnetic Field Prediction
    Z_kkm1_temp = quatmultiply(quatmultiply(quatinv(X_kkm1(1:4,idx)), [0; B_ref]), X_kkm1(1:4,idx));
    Z_kkm1(1:3,idx) = Z_kkm1_temp(2:4,1);
    
    % Angular Velocity Prediction
    Z_kkm1(4:6,idx) = X_kkm1(5:7,idx) + X_kkm1(8:10,idx);
    
end
end

function results = Kinematics(x, I, torque_c)
q = x(1:4);
w = x(5:7);
I = diag(I);

q_dot = 0.5.*[0 -w'; w -skew_matrix(w)]*q;
w_dot = I^(-1)*(-skew_matrix(w)*(I*w) + torque_c);
w_bias_dot = [0; 0; 0];

results = [q_dot; w_dot; w_bias_dot];
end

function output = skew_matrix(x)
% Returns the skew symmetric matrix of the input vector

output = [0 -x(3) x(2);...
    x(3) 0 -x(1);...
    -x(2) x(1) 0];
end

function qres = quatmultiply(q, r)
q = q';
r = r';
% Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
vec = [q(:,1).*r(:,2) q(:,1).*r(:,3) q(:,1).*r(:,4)] + ...
         [r(:,1).*q(:,2) r(:,1).*q(:,3) r(:,1).*q(:,4)]+...
         [ q(:,3).*r(:,4)-q(:,4).*r(:,3) ...
           q(:,4).*r(:,2)-q(:,2).*r(:,4) ...
           q(:,2).*r(:,3)-q(:,3).*r(:,2)];

% Calculate scalar portion of quaternion product
% scalar = s1*s2 - dot(v1,v2)
scalar = q(:,1).*r(:,1) - q(:,2).*r(:,2) - ...
             q(:,3).*r(:,3) - q(:,4).*r(:,4);
    
qres = [scalar  vec]';
end

function qinv = quatinv(qin)

q_conj = [qin(1); -qin(2:4)];
qinv = q_conj./sqrt(sum(qin.^2));

end

function rvec = RotateVecSensor2Cont(vec, q_s_c)

rvec_temp = quatmultiply(quatmultiply(quatinv(q_s_c), [0; vec]), q_s_c);
rvec = rvec_temp(2:4,1);

end

function rvec = RotateVecCont2Sensor(vec, q_s_c)

rvec_temp = quatmultiply(quatmultiply(q_s_c, [0; vec]), quatinv(q_s_c));
rvec = rvec_temp(2:4,1);

end