#!/usr/bin/env python3

#This code gives information about start time, end time, duration & topic names for each rosbag contained in the folder whose location is given in bags_path.

from importlib_metadata import metadata
import rosbag
import rospy
import os
import re
import sys

bags_path = '/home/amansharma/Downloads/output/'
bags_list = os.listdir(bags_path)
convert = lambda text : int(text) if text.isdigit() else text.lower()
al_key = lambda key : [convert(c) for c in re.split('([0-9]+)', key)]
bags_list = sorted(bags_list, key=al_key)
# print(len(bags_list))
# print(bags_list)
prev_len = 0
for name in range(len(bags_list)):
    load_path = bags_path + bags_list[name]   
    bag = rosbag.Bag(load_path, 'r')
    print('Bag name : ', bags_list[name])
    print('Bag start : ', bag.get_start_time())
    print('Bag end: ', bag.get_end_time())
    print('Bag duration: ', bag.get_end_time() - bag.get_start_time(), 'secs')
    # for topics, msgs, t in bag.read_messages():
    topic_names = list(bag.get_type_and_topic_info()[1].keys())
    print(topic_names)
    print("End of Rosbag\n\n")
    bag.close()


print("All info displayed")
sys.exit()