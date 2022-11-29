#!/usr/bin/env python3

from genericpath import isfile
from math import floor
from bagpy import bagreader
import pandas as pd
import numpy as np
import os
import rospy 
import sys
    
class operational_analytics:
    def __init__(self):
        self.duration = None
        try:
            name = rospy.get_param('~name')
            pdir_path = os.getcwd()
            self.bag_name = pdir_path.replace(os.sep,'/') + '/' + name
        except Exception:
            print('No Bag name specified!')
            sys.exit(1)
        
        try:
            self.start_time = rospy.get_param('~start')
        except Exception:
            print('Choosing default start time value of 0.0 secs\n')
            self.start_time = 0.0
        
        try:
            self.end_time = rospy.get_param('~interval')
        except Exception:
            print('Choosing default start time value of 10.0 secs\n')
            self.end_time = 10.0
        self.set_rosbag_params()
        
        try:
            self.create_csv = rospy.get_param('~create')
            if ((self.create_csv == 'T' or self.create_csv == 't')):
                self.create_topics_csv()
            else:
                print('Assuming that the required CSV files are already present!\n')
        except Exception:
            print('Assuming that the required CSV files are already present!\n')

        
        try:
            self.cal_vel_info = rospy.get_param('~vel_info')
            if (self.cal_vel_info == 'T' or self.cal_vel_info == 't'):
                print("========================================Velocity Analytics===========================================\n\n")
                self.vel_info_cal()
        except Exception:
            print('Not calculating Avg. Velocity Info.\n')

        try:
            self.cal_auto_mode_info = rospy.get_param('~mode_info')
            if (self.cal_auto_mode_info == 'T' or self.cal_auto_mode_info == 't'):
                print("========================================Operational Mode Analytics===================================\n\n")
                self.auto_info_cal()
        except Exception:
            print('Not calculating Auto Mode Info.\n')

        try:
            self.cal_auto_mode_info = rospy.get_param('~trans_info')
            if (self.cal_auto_mode_info == 'T' or self.cal_auto_mode_info == 't'):
                print("========================================Transmission Analytics=======================================\n\n")
                self.trans_info_cal()
        except Exception:
            print('Not calculating Transmission Info.\n')


    def trans_info_cal(self):
        file_name = self.bag.datafolder + '/transmission.csv'
        if  os.path.isfile(file_name):
            df = pd.read_csv(file_name)
            df = df.loc[(df['Time'] >= self.start_time) & (df['Time'] <= self.end_time)]
            vals = (sorted(df['data'].unique()))
            # print("0    =>      Park")
            # print("1    =>      Reverse")
            # print("2    =>      Neutral")
            # print("3    =>      Drive\n\n")
            total_msgs = df.shape[0]
            for val in vals:
                if val == 0:
                    print("Park        =>      ", ((df['data'].value_counts()[0]*100)/total_msgs), " %")
                elif val == 1:
                    print("Reverse     =>      ", ((df['data'].value_counts()[1]*100)/total_msgs), " %")
                elif val == 2:
                    print("Neutral     =>      ", ((df['data'].value_counts()[2]*100)/total_msgs), " %")
                elif val == 3:
                    print("Drive       =>      ", ((df['data'].value_counts()[3]*100)/total_msgs), " %\n")
                else:
                    print("Unkown Trsnamission mode.")

            rev = df.iloc[:,1] 
            isrev = rev.eq(1)
            mask = isrev & (~(isrev.shift() & isrev.shift(-1)) )
            rev_engaged = list(rev.index[mask])
            rev_engaged_counts = floor(len(rev_engaged)/2)
            print("No. of times the vehicle was reversed: ", int(rev_engaged_counts), "\n\n")
        else:
            print('Required File does not exist\n')

    def vel_info_cal(self):
        file_name = self.bag.datafolder + '/velocity.csv'
        if  os.path.isfile(file_name):
            msg_freq = self.topic_table.loc[self.topic_table['Topics'] == '/velocity']['Frequency'].item()
            duration = self.end_time - self.start_time
            total_time = duration * msg_freq
            df = pd.read_csv(file_name)
            df = df.loc[(df['Time'] >= self.start_time) & (df['Time'] <= self.end_time)]
            # print(df)
            vel = df.iloc[:,1]
            moving = np.count_nonzero(vel)
            perc = (moving/total_time) * 100
            print('Percentage of time the vehicle was moving: ', perc, ' % of the time')
            sum_vel = vel.sum()
            avg_vel = sum_vel/total_time
            print('Average Velocity of trip : ', avg_vel, ' m/s\n')

            isstop = vel.eq(0)
            mask = isstop & (~(isstop.shift() & isstop.shift(-1)) )
            stops = list(vel.index[mask])
            no_stops = floor(len(stops)/2)
            print("No. of times the vehicle came to a complete stop: ", int(no_stops), "\n\n")
        else:
            print('Required File does not exist\n')

    def auto_info_cal(self):
        file_name = self.bag.datafolder + '/vehiclemode.csv'
        if os.path.isfile(file_name):
            # print("0        =>      OFF     =>      Vehicle is offline")
            # print("1        =>      INITIALIZE      =>      Vehicle autonomy is starting (during bootup usually)")
            # print("2        =>      INACTIVE        =>      Vehicle autonomy is inactive (usually right after bootup)")
            # print("3        =>      INACTIVE        =>      Vehicle is shutting down")
            # print("4        =>      READY_IDLE      =>      Vehicle is idle: Not in driving mode")
            # print("5        =>      READY_HUMAN     =>      Vehicle is in manual drive mode. Any motion in this state is manual")
            # print("6        =>      READY_ROBOT     =>      Vehicle is in autonomous mode. Any motion in this state is autonomous\n\n")
            df = pd.read_csv(file_name)
            df = df.loc[(df['Time'] >= self.start_time) & (df['Time'] <= self.end_time)]
            # print(mode)
            total_msgs = df.shape[0]
            try:
                offline_count = df['data'].value_counts()[0]
                offline_perc = (offline_count/total_msgs) * 100
                print('Percentage of time vehicle was in Autonomous Mode : ', offline_perc, '%')
            except Exception:
                print('Vehicle was not offline at all in the given duration.')
            try:
                auto_count = df['data'].value_counts()[6]
                auto_perc = (auto_count/total_msgs) * 100
                print('Percentage of time vehicle was in Autonomous Mode : ', auto_perc, '%')
            except Exception:
                print('Vehicle did not work in Autonomous Mode at all in the given duration.')

            try:
                manual_count = df['data'].value_counts()[5]
                manual_perc = (manual_count/total_msgs) * 100
                print('Percentage of time vehicle was in Manual Mode : ', manual_perc, '%')
            except Exception:
                print('Vehicle did not work in Manual Mode at all in the given duration.')

            try:
                xdf = df.loc[(df['data'] != 5) & (df['data'] != 6)]
                idle_count = xdf.shape[0]
                idle_perc = (idle_count/total_msgs) * 100
                print('Percentage of time vehicle was in Idle/Bootup Mode : ', idle_perc, '%\n')
            except Exception:
                print("Vehicle wa not in Idle/Bootup Mode at all in the given duration.\n")

            mode = df.iloc[:,1]
            isman = mode.eq(5)
            mask = isman & (~(isman.shift() & isman.shift(-1)) )
            engages = list(mode.index[mask])
            man_engages_count = floor(len(engages)/2)
            print("No. of times Manual Mode was Engaged: ", int(man_engages_count), "\n\n")

        else:
            print('Required File does not exist')

    def set_rosbag_params(self):
        self.bag = bagreader(self.bag_name)
        self.topic_table = self.bag.topic_table
        try:
            self.topics = rospy.get_param('~topics')
        except Exception:
            print('Choosing all topics to generate csv')
            self.topics = list(self.bag.topics)
        self.start_time = self.start_time + self.bag.start_time
        self.end_time = self.end_time + self.start_time
        self.bag_duration = self.bag.end_time - self.bag.start_time
        if (self.end_time - self.start_time) > self.bag_duration:
                print('Capping end time to duration of rosbag')
                self.end_time = self.bag.start_time + self.bag_duration
        # print(self.start_time)
        # print(self.end_time)
        # print(self.topics)

    def create_topics_csv(self):
        csv_files = []
        for t in self.topics:
            # print(t)
            data = self.bag.message_by_topic(t)
            csv_files.append(data)

        print(csv_files[0])

def main():
    rospy.init_node('Operational_Analytics',anonymous=True)
    map = operational_analytics()
    # rospy.spin()

if __name__ == '__main__':
    main()




# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=10 _interval:=40 _topics:="['/tf','/transmission','/vehiclemode','/velocity']" _create:='F'
# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=10 _interval:=40 _topics:="['/velocity']"
# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=10 _interval:=40 _topics:="['/tf','/transmission','/vehiclemode','/velocity']" _create:='F' _cal_vel:='T'
# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=1750 _interval:=1800 _create:='F' _cal_vel:='T'
# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=1700 _interval:=1800  _create:='F' _cal_vel:='T' _cal_auto:='T'



# rosrun reg operational_analytics.py _name:='2022-07-28-14-11-09.bag' _start:=0 _interval:=1800  _create:='F' _vel_info:='T' _mode_info:='T'
# rosrun reg operational_analytics.py _name:='sample.bag' _start:=0 _interval:=600 _create:='T' _vel_info:='T' _mode_info:='T' _topics:="['/actuation','/gps','/imu','/tf','/transmission','/vehiclemode','/velocity']"