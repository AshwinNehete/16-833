% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2019 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

dx = lx-rx;
dy = ly-ry;
q = dx^2+dy^2;

dt_drx = dy/q;
dt_dlx = -dy/q;
dt_dry = -dx/q;
dt_dly = dx/q;

dd_drx = -dx/sqrt(q);
dd_dlx = dx/sqrt(q);
dd_dry = -dy/sqrt(q);
dd_dly = dy/sqrt(q);

H(1,1) = dt_drx;
H(1,2) = dt_dry;
H(1,3) = dt_dlx;
H(1,4) = dt_dly;

H(2,1) = dd_drx;
H(2,2) = dd_dry;
H(2,3) = dd_dlx;
H(2,4) = dd_dly;

end