clear;clc;

sub = rossubscriber("/tracking","sensor_msgs/Image","DataFormat","struct");
while true
    msg = receive(sub);
    img = rosReadImage(msg);
    imshow(img)
end

