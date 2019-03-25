% SOLVE_QR2
% 16-833 Spring 2019 - *Stub* Provided
% Solves linear system using second QR method
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using the specified
%             version of the QR decomposition
%     R     - R factor from the QR decomposition
%
function [x, R] = solve_qr2(A, b)

[~, n] = size(A);
[C, R, e] = qr(A, b, 'vector');

R = R(1:n, :);
C = C(1:n);
x(e) = back_sub(R, C);
x = x';

end