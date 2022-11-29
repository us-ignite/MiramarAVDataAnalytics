#!/usr/bin/env python3
from turtle import color
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
import pandas as pd
import folium
import webbrowser
import time

class plot_gps:
    def __init__(self):
        self.n = 0
        self.start_time = time.time()
        # Param to decide the duration of rosbag till which we want to draw the route for. By default, only first 10 secs is used to plot the route if no param is given.
        try:
            self.map_duration = rospy.get_param('~duration') - 10.0
        except Exception:
            print('Choosing default value of 10.0 secs')
            self.map_duration = 10.0

        # Param to allow displaying the route followed by vehicle in color-coded manner.
        try:
            self.with_mode = rospy.get_param('~with_mode')
        except Exception:
            self.with_mode = 'f'
            print("Displaying route without considering Vehicle mode")

        # Variables to store the previous & the present gps coordinates in order to draw a polyline between the points.
        self.old_location = None
        self.new_location = None

        # Variables to define layers on the map to plot various analytics parameters like velocity, vehiclemode etc.
        self.gp = None
        self.man_gp = folium.FeatureGroup(name='Manual Mode')
        self.auto_gp = folium.FeatureGroup(name='Autonomous Mode')

        # Subscriber for vehiclemode topic
        # if self.with_mode == 'T' or self.with_mode == 't':
        self.mode_topic = '/vehiclemode'
        self.mode_sub = rospy.Subscriber(self.mode_topic, UInt32, self.get_vehicle_mode)

        # Subscriber for Velocity topic
        self.vel_topic = '/velocity'
        self.vel_sub = rospy.Subscriber(self.vel_topic, Float64, self.get_velocity)
        
        # Subsciber for GPS topic
        self.gps_topic = '/gps'
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.plot)

    def get_vehicle_mode(self,msg):
        self.vehicle_mode = int(msg.data)
        # print(type(self.vehicle_mode))

    def get_velocity(self,msg):
        self.velocity = float(msg.data)

    # Function to generate the map with all the requested information requested
    def plot(self, msg):
        # Starts a map with the intial GPS coordinates and adds a marker for the staring point
        if self.n == 0:
            self.map = folium.Map(location=[msg.latitude,msg.longitude], zoom_start=20)
            # self.map = folium.add_categorical_legend(self.map, 'Vehicle Mode Categories', colors = ['red','green','blue'], labels = ['Manual', 'Autonomous', ['Others/ Non-color coded']])
            folium.Marker(location=[msg.latitude,msg.longitude], icon=folium.Icon(color="green")).add_to(self.map)
            self.n += 1
            self.old_location = [msg.latitude,msg.longitude]
        
        # If the duration for map generation is finishing then adds the marker for the end point & adds the child layers to display other topics data on the map. Finally, saves the map as an HTML file.
        elif (self.map_duration < (time.time() - self.start_time) < (self.map_duration + 0.2)) and (self.n!=0) :
            self.n = -1
            folium.Marker(location=[msg.latitude,msg.longitude], icon=folium.Icon(color="red")).add_to(self.map)
            self.map.add_child(self.man_gp)
            self.map.add_child(self.auto_gp)
            self.map.add_child(folium.LayerControl())
            # print("Saving map")
            # print(self.map)
            self.map.save("Map.html")
            webbrowser.open_new_tab("Map.html")

        # If the messages are in the between the required interval, add information & draw polylines between coordinates based on the info received from various topics being published.
        else:
            
            if self.with_mode == 'T' or self.with_mode == 't':
                if (self.vehicle_mode == 5):
                    self.color = "red"
                    self.gp = self.man_gp
                elif (self.vehicle_mode == 6):
                    self.color = "green"
                    self.gp = self.auto_gp
                else:
                    self.color = "blue"
                    self.gp = self.map
            else:
                self.color = 'blue'
                self.gp = self.map
            
            self.new_location = [msg.latitude,msg.longitude]
            loc = [self.old_location,self.new_location]
            folium.PolyLine(loc, color= self.color, weight=5, opacity=0.8, no_clip=True, smooth_factor = 0.1, tooltip=self.velocity).add_to(self.gp)
            # print(self.velocity)
            self.old_location = self.new_location
            

def main():
    rospy.init_node('plot_map',anonymous=True)
    map = plot_gps()
    rospy.spin()

if __name__ == '__main__':
    main()