function Obstacle_Area = genarea()
% Interest area value >0
% Obstacle area value =0

I_raw=imread('./maps/image2.png');              % map here

I_raw=imresize(I_raw,[101,101]);    % resize

I_gray=rgb2gray(I_raw);
Obstacle_Area=im2double(I_gray);
Obstacle_Area=Obstacle_Area*255;

%[obs_row, obs_col] = find(Obstacle_Area == 0);
%figure;
%imshow(I_gray);
%figure;
%image(Area1);