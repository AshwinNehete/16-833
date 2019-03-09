%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

num_landmark = length(measure)/2.0;
landmark = zeros(num_landmark * 2.0, 1);
landmark_cov = zeros(2 * num_landmark, 2 * num_landmark);

for i = 0:num_landmark-1
    beta = measure(2*i+1);
    distance = measure(2*i+2);
    
    % estimate position
    landmark(2*i+1) = pose(1) + distance*cos(pose(3) + beta);
    landmark(2*i+2) = pose(2) + distance*sin(pose(3) + beta);
    
    % estimate covariance
    H_init = [1 0 -distance*sin(pose(3) + beta); 0 1 distance*cos(pose(3) + beta)];
    Q_init = [-distance * sin(pose(3) + beta) cos(pose(3) + beta); distance * cos(pose(3) + beta) sin(pose(3) + beta)];
    landmark_cov(2*i+1:2*i+2, 2*i+1:2*i+2) = H_init*pose_cov*H_init' + Q_init*measure_cov*Q_init';
    
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*num_landmark) ; zeros(2*num_landmark, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    F = [eye(3) zeros(3, 2*num_landmark)];
    G = eye(3 + 2*num_landmark) + F' * [0 0 (-d*sin(x(3))); 0 0 (d * cos(x(3))); 0 0 0] * F;
    V = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];
    
    % convert covariance to world frame
    R = V*control_cov*V';
    % predict the mean @ t + 1
    x_pre = x + F' * [d * cos(x(3)); d * sin(x(3)); alpha];
    % predict the covariance @ t + 1
    P_pre = G * P * G' + F' * R * F;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    
    for j = 0:num_landmark-1
        meas = measure(2*j+1:2*j+2);
        landmark = x_pre(3+2*j+1:3+2*j+2);
        % predict measurement using state
        delta_x = landmark(1) - x_pre(1);
        delta_y = landmark(2) - x_pre(2);
        q = (delta_x^2 + delta_y^2)^0.5;
        meas_pre = [wrapToPi(atan2(delta_y, delta_x) - x_pre(3)); q];
        
        % predict covariance of measurement
        Fm = zeros(5, 3+2*num_landmark);
        Fm(1:3,1:3) = eye(3);
        Fm(4:5, 3+2*j+1:3+2*j+2) = eye(2);
        
        H = [delta_y / (q ^ 2) -delta_x / (q ^ 2) -1 -delta_y / (q ^ 2) delta_x / (q ^ 2); 
            -delta_x / q -delta_y / q 0 delta_x / q delta_y / q] * Fm;
        S = H * P_pre * H' + measure_cov;
        
        % since we have both the predicted measurement and the actual
        % measurement, we can do the update accordingly
        K = P_pre * H' * inv(S);
        x_pre = x_pre + K * (meas - meas_pre);
        P_pre = (eye(3 + 2 * num_landmark) - K * H) * P_pre;
    end
    
    % update @ this timestamp is done, update the state
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% plot ground truth
ground_truth = [3 6 3 12 7 8 7 14 11 6 11 12];
for gt_index = 0 : num_landmark - 1
    landmark_gt = ground_truth(gt_index * 2 + 1 : gt_index * 2 + 2);
    plot(landmark_gt(1), landmark_gt(2), '*r');
end

est_landmark = x(4:end);
est_landmark_cov = P(4:end, 4:end);

for i = 0:num_landmark-1
    gt = ground_truth(2*i+1:2*i+2);
    est = est_landmark(2*i+1:2*i+2);
    
    delta_x = gt(1)-est(1);
    delta_y = gt(2)-est(2);
    euclidian_dist = (delta_x^2+delta_y^2)^0.5;
    
    est_cov = est_landmark_cov(2*i+1:2*i+2, 2*i+1:2*i+2);
    val = [delta_x delta_y];
    mahalanobis_dist = (val * est_cov * val')^0.5;
    
    disp(['[LANDMARK_' num2str(i + 1) ']:']);
    disp(['Euclidean: ' num2str(euclidian_dist) '; Mahalanobis: ' num2str(mahalanobis_dist)]);
    
end

%==== Close data file ====
fclose(fid);
