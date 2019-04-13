im = imread('Calibration.jpg');
im = imresize(im, 0.25);

% use harris edge detection
C = detectHarrisFeatures(rgb2gray(im)); % change to harris corner
imshow(im);
hold on
% take only the 240 strongest corner features
C = C.selectStrongest(240).Location;
xs = C(:,1);
ys = C(:,2);
plot(xs, ys,'r*');
% plot(497,451, 'go'); % image center gotten form K matrix

% hard coded 3d coordinates

% coord3d = [1 1 0; 2 2 0; 2 0 1 ; 2 0 2; 3 0 3; 0 2 0]; % Good
% coord2d = [498 435; 497 410; 408 495; 408 544; 379 579; 573 436];

coord3d = [2 0 4; 4 0 4; 0 2 4; 0 4 4; 2 1 0; 1 2 0]; % good
coord2d = [414 638; 348 599; 572 634; 633 592; 460 422; 529 421];

% select points manually
% points = ginput(6);
% coord2d = [];

% for i=1:size(points,1)
%     idx = knnsearch(C,points(i,:));
%     coords = C(idx, :);
%     coord2d = [coord2d; coords];
%     plot(coords(1), coords(2), 'bo');
% end



[K R t] = cameracali(coord2d, coord3d);
disp(K)