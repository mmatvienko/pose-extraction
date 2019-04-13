target = imread('TargetImage.jpg');
source = imread('SourceImage.jpg');

subplot(1,2,1), imshow(target);
subplot(1,2,2), imshow(source);

% 8 colors used for marking correspondences
colors = ['b', 'k', 'r', 'g', 'y', 'c', 'm', 'w'];

% points = ginput(16);
% 
% targets = points(1:8,:);
% sources = points(9:16,:);

% points selected using ginput(16)
sources = [1334 1178; 2049 1075; 2379 1056; 3461 924; 789 1734; 1701 1442; 1918 1414; 986 1997];
targets = [1273 1084; 2167 1047; 2553 1018; 3729 915; 1141 1715; 1781 1395; 2111 1395; 3061 2035];

subplot(1,2,1), imshow(target);
for i=1:8
    hold on;
    subplot(1,2,1), plot(targets(i,1), targets(i,2), strcat('o', colors(i)));
end

subplot(1,2,2), imshow(source);
for i=1:8
    hold on;
    subplot(1,2,2), plot(sources(i,1), sources(i,2), strcat('x', colors(i)));
end

% K value from previous part
K = [56.7311   -0.8954  497.8304;
     0         55.4398  451.8533;
     0         0        1.00000];

[R, T] = relativepose(sources, targets, K);