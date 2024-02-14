%% Area of shadow on camera sensor
clc;clear;close all
% Ls = 0.083; % Edge length of the sq. plate (m)
As = 0.083*0.083; % Area of the sq. plate (m^2)
hl = 0.2; % Height of the light source from deck (m)
f = 0.00083; % Focal length of the lens (m)
p = 3e-6; % Pixelpitch (m)
h = 0.05:0.01:1.5; % Height of the UAV (m)
k = 1;

% while k <= length(h)
%     L(k,1) = Ls*(1 + (h(k)/hl)); % Original edge length of the shadow
%     l(k,1) = L(k,1)*f/h(k)*1000; % Edge length of shadow on sensor
%     k = k+1;
% end
while k <= length(h)
    A(k,1) = As*(1 + (h(k)/hl)); % Original area of the shadow
    a(k,1) = A(k,1)*f/h(k); % Area of shadow on sensor
    k = k+1;
end

% time = linspace(0,5,length(h));
% L_dot = mean(diff(L))/mean(diff(time)); % rate of change of length of the shadow (m/s)
% h_dot = mean(diff(h))/mean(diff(time)); % velocity of camera (m/s)
% number_of_pixels = round(l/p);
% 
% figure(1)
% plot(h,a)
% title("Area of shadow on camera sensor")
% xlabel("height(m)")
% ylabel("lenght(mm)")
% grid on
% grid minor
% 
% figure(2)
% plot(h,number_of_pixels)
% title("No. of pixels covered by the edge")
% xlabel("height(m)")
% ylabel("number of pixels")
% grid on
% grid minor
%%
load("camera_params.mat")
images = ["1.jpg" "2.jpg" "3.jpg" "4.jpg" "5.jpg" "6.jpg" "7.jpg" "8.jpg"...
         "9.jpg" "10.jpg" "11.jpg" "12.jpg"];

for i = 1:length(images)
    img = imread(images(i));
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
    area(i,1) = stats(2)*(p^2)*1000;
end