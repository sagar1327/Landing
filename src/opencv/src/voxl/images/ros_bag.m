clear;clc;

bag = rosbag("bag/new_tracking_record.bag");
mssg = readMessages(bag,"DataFormat","struct");

for i = 1:length(mssg)
    img = rosReadImage(mssg{i,1});
    imshow(img)
end