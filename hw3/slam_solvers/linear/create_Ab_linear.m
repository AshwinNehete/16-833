% CREATE_AB_LINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
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
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first 
% pose
sigma_o = sqrtm(inv(sigma_o));
sigma_l = sqrtm(inv(sigma_l));

A(1,1) = 1;
A(2,2) = 1;
b(1:2, 1) = [0,0];

landmark_state_index_offset = n_poses;

for odom_index = 0:n_odom-1
    odometry = odom(odom_index+1,:);
    
    b(2+odom_index*2+1:2+odom_index*2+2) = sigma_o*odometry';
    
    odomJ = [-1 0 1 0; 0 -1 0 1];
    odomJ = sigma_o * odomJ;
    
    A(2 + odom_index * 2 + 1, odom_index * 2 + 1) = odomJ(1, 1);
    A(2 + odom_index * 2 + 1, odom_index * 2 + 2) = odomJ(1, 2);
    A(2 + odom_index * 2 + 1, (odom_index + 1) * 2 + 1) = odomJ(1, 3);
    A(2 + odom_index * 2 + 1, (odom_index + 1) * 2 + 2) = odomJ(1, 4);
    A(2 + odom_index * 2 + 2, odom_index * 2 + 1) = odomJ(2, 1);
    A(2 + odom_index * 2 + 2, odom_index * 2 + 2) = odomJ(2, 2);
    A(2 + odom_index * 2 + 2, (odom_index + 1) * 2 + 1) = odomJ(2, 3);
    A(2 + odom_index * 2 + 2, (odom_index + 1) * 2 + 2) = odomJ(2, 4);
end

obs_A_row_offset = p_dim*(n_odom+1);
for obs_index = 0 : n_obs - 1
    pose_index = obs(obs_index + 1, 1);
    landmark_index = obs(obs_index + 1, 2);
    measurement = obs(obs_index + 1, 3 : 4);
    
    b(obs_A_row_offset + obs_index * 2 + 1 : obs_A_row_offset + obs_index * 2 + 2) = sigma_l * measurement';
    
    measJ = [-1 0 1 0; 0 -1 0 1];
    measJ = sigma_l * measJ;

    A(obs_A_row_offset + obs_index * 2 + 1, pose_index * 2 - 1) = measJ(1, 1);
    A(obs_A_row_offset + obs_index * 2 + 1, pose_index * 2) = measJ(1, 2);
    A(obs_A_row_offset + obs_index * 2 + 1, (landmark_state_index_offset + landmark_index) * 2 - 1) = measJ(1, 3);
    A(obs_A_row_offset + obs_index * 2 + 1, (landmark_state_index_offset + landmark_index) * 2) = measJ(1, 4);
    A(obs_A_row_offset + obs_index * 2 + 2, pose_index * 2 - 1) = measJ(2, 1);
    A(obs_A_row_offset + obs_index * 2 + 2, pose_index * 2) = measJ(2, 2);
    A(obs_A_row_offset + obs_index * 2 + 2, (landmark_state_index_offset + landmark_index) * 2 - 1) = measJ(2, 3);
    A(obs_A_row_offset + obs_index * 2 + 2, (landmark_state_index_offset + landmark_index) * 2) = measJ(2, 4);
end

% Make A a sparse matrix 
As = sparse(A);