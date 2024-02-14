clc;clear;
load("camera_params.mat")

As = 0.083*0.083; % Area of the sq. plate (m^2)
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)

img = imread("frame0001.jpg");
img = undistortFisheyeImage(img,Params.Intrinsics);
gray = rgb2gray(img);
T = adaptthresh(gray,"Statistic","gaussian");
bw = imbinarize(gray,T);
mask = ~bw;
imshow(mask)
CC = bwconncomp(mask);
stats = regionprops(CC,'Area');
stats = vertcat(stats.Area);
stats = sort(stats,"descend");
area = stats(2)*(p^2)*1000;
h = (As*f*hl)/(area*hl - As*f);

