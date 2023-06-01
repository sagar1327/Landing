#!/usr/bin/env python

import argparse
import os

parser = argparse.ArgumentParser(description='images to video')
parser.add_argument('output', type=str, help='output name, example: xyz.mp4')
parser.add_argument('frame_rate', type=int, help='output video frame rate')
args = parser.parse_args()
 
path = '~/ROS/src/ros_bag/images'
command = "ffmpeg -framerate {frame_rate} -i {path}/frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p {output_name}".format(frame_rate=args.frame_rate, path=path, output_name=args.output)

os.system('mv ~/.ros/frame*.jpg {}'.format(path))

if os.getcwd()==path:
	os.system(command)
else:
	os.system('cd {} && {}'.format(path, command))
