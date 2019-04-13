function [K, R, t] = cameracali(coord2d, coord3d)
M = [];
% build M matrix of form :
% x y z 1 0 0 0 0 -u*x -u*y -u*z -u
% 0 0 0 0 x y z 1 -v*x -v*y -v*z -v
% where x y z are the world coords
% and u v are image coordinates
for i=1:size(coord3d, 1)
    x = coord3d(i, 1);
    y = coord3d(i, 2);
    z = coord3d(i, 3);
    
    u = coord2d(i,1);
    v = coord2d(i,2);
    
    M = [M;[x y z 1 0 0 0 0 -u*x -u*y -u*z -u];[0 0 0 0 x y z 1 -v*x -v*y -v*z -v]];
    
end

% svd decompose
[~, ~, V] = svd(M);
% take last column and turn it into a 3x4 matrix
pi = reshape(V(:, 12), [4 3])';
% satisfy the condition that ||pi^s||^2 = 1
pi = pi/norm(pi);

if det(pi(1:3,1:3)) < 0
    pi = -pi;
end

% build mu row vectors for decomp of \Pi to get K
mu1 = pi(1,:)';
mu2 = pi(2,:)';
mu3 = pi(3,:)';

k = zeros([3 3]);

% compute Gram-Schmidt decomposition
k(3,3) = norm(mu3);
r3 = mu3 / k(3,3);
k(2,3) = mu2'*r3;
k(2,2) = norm(mu2 - k(2,3)*r3);
r2 = (mu2 - k(2,3)*r3) / k(2,2);
k(1,3) = mu1'*r3;
k(1,2) = mu1'*r2;
k(1,1) = norm(mu1 - k(1,2)*r2 - k(1,3)*r3);
r1 = (mu1 - k(1,2)*r2 - k(1,3)*r3) / k(1,1);

% build K, construct R, and get t... pretty obvious
K = k/k(3,3);
R = [r1';r2';r3'];
t = K^-1*pi(:,4);
end
