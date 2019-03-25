% ERROR_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)
% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);    % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

% Initialize error
err = 0;

for odom_index = 1:n_odom
    % retrieve infomation
    odometry = odom(odom_index, :);
    odom_prev = x(2 * odom_index - 1 : 2 * odom_index);
    odom_cur = x(2 * (odom_index + 1) - 1 : 2 * (odom_index + 1));
    
    est = meas_odom(odom_prev(1), odom_prev(2), odom_cur(1), odom_cur(2));
 
    odom_err = odometry - est';
    err = err + odom_err(1) ^ 2 + odom_err(2) ^ 2;
end

% error for observations
landmark_state_index_offset = n_poses;
for obs_index = 1 : n_obs
    % retrieve infomation
    pose_index = obs(obs_index, 1);
    landmark_index = obs(obs_index, 2);
    measurement = obs(obs_index, 3 : 4);
    
    % compute the estimated bearing angle and range according to the state
    pose_est = x(pose_index * 2 - 1 : pose_index * 2);
    landmark_est = x((landmark_state_index_offset + landmark_index) * 2 - 1 : (landmark_state_index_offset + landmark_index) * 2);
    est = meas_landmark(pose_est(1), pose_est(2), landmark_est(1), landmark_est(2));
    
    err = err + wrapToPi(measurement(1) - est(1)) ^ 2 + (measurement(2) - est(2)) ^ 2;
end

end

