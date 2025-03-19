#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math
from pynput import keyboard
import copy

class SteeringSpeedNode(Node):
    def __init__(self):
        super().__init__("gap_steering_node")
        self.get_logger().info("the steering node has started")
        self.scan_param = self.declare_parameter("scan_topic", "/scan")
        self.sub_scan = self.create_subscription(LaserScan, self.scan_param.get_parameter_value().string_value, self.filter_scan_cb, 10)
        self.filtered_scan_param = self.declare_parameter("filtered_scan_topic", "/filtered_scan")
        self.filtered_laser_scan_pub = self.create_publisher(LaserScan, self.filtered_scan_param.get_parameter_value().string_value, 10)
        self.drive_param = self.declare_parameter("drive_topic", "/drive")
        self.pub_vel_cmd = self.create_publisher(AckermannDriveStamped, self.drive_param.get_parameter_value().string_value, 10)
        self.tmr = self.create_timer(0.01, self.follow_the_gap)
        self.limit_angle_param = self.declare_parameter("limit_angle", 70.0)
        self.limit_angle = self.limit_angle_param.get_parameter_value().double_value # in degrees
        self.scan_msg = LaserScan()
        self.filtered_scan_msg = LaserScan()
        self.vel_cmd = AckermannDriveStamped()
        self.obs_thresh_param = self.declare_parameter("obstacle_distance_thresh", 0.8)
        self.obs_thresh = self.obs_thresh_param.get_parameter_value().double_value     
        self.theta = 0.0  
        self.possible_edges = []
        self.d_1 = None
        self.d_2 = None
        self.close_rays_thresh_param = self.declare_parameter("close_rays_thresh", 170)
        self.close_rays_thresh = self.close_rays_thresh_param.get_parameter_value().integer_value
        self.far_rays_thresh_param = self.declare_parameter("far_rays_thresh", 50)
        self.far_rays_thresh = self.far_rays_thresh_param.get_parameter_value().integer_value
        self.min_distance_param = self.declare_parameter("min_distance", 5.0)
        self.min_distance = self.min_distance_param.get_parameter_value().double_value
        self.max_distance_param = self.declare_parameter("max_distance", 9.0)
        self.max_distance = self.max_distance_param.get_parameter_value().double_value
        self.close_edges_thresh_param = self.declare_parameter("close_edges_thresh", 0.15)
        self.close_edges_thresh = self.close_edges_thresh_param.get_parameter_value().double_value
        self.kp_param = self.declare_parameter("kp", 1.0)
        self.kp = self.kp_param.get_parameter_value().double_value

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

    def on_press(self, key):
        try:
            if key.char == 's':
                # self.start_algorithm = True
                self.vel_cmd.drive.speed = -0.8
            if key.char == 'w':
                self.vel_cmd.drive.speed = 1.8
            if key.char == 'b':
                self.vel_cmd.drive.speed = 6.0
        except AttributeError:
            self.get_logger().warn("error while sending.. :(")

    def on_release(self, key):
        # Stop the robot when the key is released
        # self.start_algorithm = False
        self.vel_cmd.drive.speed = 0.0
        if key == keyboard.Key.esc:
            # Stop listener
            return False
        
    def find_possible_edges(self):
        ranges = self.scan_msg.ranges
        possible_edges = []
        for i in range(self.smaller_angle_index, self.bigger_angle_index-1):
            if abs(ranges[i] - ranges[i+1]) > self.obs_thresh: # we found an edge (index, distance, angle)
                angle = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
                possible_edges.append((i, min(ranges[i], ranges[i+1]), angle))
        return possible_edges

    def get_theta_target(self, poss_edges):
        if len(poss_edges) == 0:
            return 0.0
        # find the most dangerous edge
        self.filtered_scan_msg = copy.deepcopy(self.scan_msg)
        smallest_dist = poss_edges[0][1]
        dangerous_edge = poss_edges[0]
        for curr_edge in poss_edges:
            if smallest_dist > curr_edge[1]:
                dangerous_edge = curr_edge             
                smallest_dist = curr_edge[1]

        for i in range(-self.close_rays_thresh//2, self.close_rays_thresh//2):
            var_index = (i + dangerous_edge[0])
            if 0 <= var_index < len(self.filtered_scan_msg.ranges):
                # filter the readings
                self.filtered_scan_msg.ranges[var_index] = dangerous_edge[1]
        
        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(self.filtered_scan_msg)

        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if self.filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = self.filtered_scan_msg.ranges[i]
                theta = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
        return theta

    def get_theta_target_2(self, poss_edges):
        if len(poss_edges) == 0:
            return 0.0
        # find the most dangerous edge
        self.filtered_scan_msg = copy.deepcopy(self.scan_msg)
        smallest_dist = poss_edges[0][1]
        dangerous_edge = poss_edges[0]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge = curr_edge             
                smallest_dist = curr_edge[1]

        for i in range(-self.close_rays_thresh//2, self.close_rays_thresh//2):
            var_index = (i + dangerous_edge[0])
            if 0 <= var_index < len(self.filtered_scan_msg.ranges):
                # filter the readings
                self.filtered_scan_msg.ranges[var_index] = dangerous_edge[1]
        
        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(self.filtered_scan_msg)

        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if self.filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = self.filtered_scan_msg.ranges[i]
                theta = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
        return theta

    def get_theta_target_3(self, poss_edges:list):
        if len(poss_edges) == 0:
            return 0.0
        # find the most dangerous edge
        self.filtered_scan_msg = copy.deepcopy(self.scan_msg)
        dangerous_edge = poss_edges[0]
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge = curr_edge             
                smallest_dist = curr_edge[1]
        
        dangerous_edge_2nd = poss_edges[0]        
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge_2nd = curr_edge             
                smallest_dist = curr_edge[1]
            if dangerous_edge_2nd == dangerous_edge:
                dangerous_edge_2nd = curr_edge

        for i in range(-self.close_rays_thresh//2, self.close_rays_thresh//2):
            var_index = (i + dangerous_edge[0])
            var_index_2nd = (i + dangerous_edge_2nd[0])
            if 0 <= var_index < len(self.filtered_scan_msg.ranges):
                # filter the readings
                self.filtered_scan_msg.ranges[var_index] = dangerous_edge[1]
            if 0 <= var_index_2nd < len(self.filtered_scan_msg.ranges):
                self.filtered_scan_msg.ranges[var_index_2nd] = dangerous_edge_2nd[1]
                
        self.d_1 = dangerous_edge
        self.d_2 = dangerous_edge_2nd

        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(self.filtered_scan_msg)

        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if self.filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = self.filtered_scan_msg.ranges[i]
                theta = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
        return theta

    def get_theta_target_4(self, poss_edges:list):
        if len(poss_edges) == 0:
            return 0.0
        # find the most dangerous edge
        self.filtered_scan_msg = copy.deepcopy(self.scan_msg)
        dangerous_edge = poss_edges[0]
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge = curr_edge             
                smallest_dist = curr_edge[1]
        
        dangerous_edge_2nd = poss_edges[0]        
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge_2nd = curr_edge             
                smallest_dist = curr_edge[1]
            if dangerous_edge_2nd == dangerous_edge:
                dangerous_edge_2nd = curr_edge
                
        self.d_1 = dangerous_edge
        self.d_2 = dangerous_edge_2nd

        # check if the edge is very far away
        if self.d_1[1] > self.min_distance and self.d_2[1] > self.min_distance:
            avg_dist_x = min(self.d_1[1], self.d_2[1])
            m = (self.close_rays_thresh - self.far_rays_thresh)/(self.min_distance - self.max_distance)
            c = self.close_rays_thresh - m*self.min_distance
            self.close_rays_thresh = int(m*avg_dist_x + c)
            # cap the rays_thresh
            if self.close_edges_thresh > self.close_edges_thresh_param.get_parameter_value().integer_value:
                self.rays_radius = self.close_edges_thresh_param.get_parameter_value().integer_value
            if self.rays_radius < self.far_rays_thresh:
                self.rays_radius = self.far_rays_thresh
        else:
            self.close_rays_thresh = self.close_rays_thresh_param.get_parameter_value().integer_value

        for i in range(-self.close_rays_thresh//2, self.close_rays_thresh//2):
            var_index = (i + dangerous_edge[0])
            var_index_2nd = (i + dangerous_edge_2nd[0])
            if 0 <= var_index < len(self.filtered_scan_msg.ranges):
                # filter the readings
                self.filtered_scan_msg.ranges[var_index] = dangerous_edge[1]
            if 0 <= var_index_2nd < len(self.filtered_scan_msg.ranges):
                self.filtered_scan_msg.ranges[var_index_2nd] = dangerous_edge_2nd[1]
        
        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(self.filtered_scan_msg)

        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if self.filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = self.filtered_scan_msg.ranges[i]
                theta = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
        return theta
    
    def find_critical_two_edges(self, poss_edges:list):
        # find the most dangerous edges
        self.filtered_scan_msg = copy.deepcopy(self.scan_msg)
        dangerous_edge = poss_edges[0]
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge = curr_edge             
                smallest_dist = curr_edge[1]
        
        dangerous_edge_2nd = poss_edges[0]        
        smallest_dist = poss_edges[0][1]
        for curr_edge in poss_edges:
            # the close_edges_thresh is to make the readings more stable "reduces wobbling"
            if (smallest_dist > curr_edge[1]) and abs(smallest_dist - curr_edge[1]) > self.close_edges_thresh:
                dangerous_edge_2nd = curr_edge             
                smallest_dist = curr_edge[1]
            if dangerous_edge_2nd == dangerous_edge:
                dangerous_edge_2nd = curr_edge
                
        self.d_1 = dangerous_edge
        self.d_2 = dangerous_edge_2nd

    def filter_scan(self):
        # check if the edge is very far away
        if self.d_1[1] > self.min_distance and self.d_2[1] > self.min_distance:
            avg_dist_x = min(self.d_1[1], self.d_2[1])
            m = (self.close_rays_thresh - self.far_rays_thresh)/(self.min_distance - self.max_distance)
            c = self.close_rays_thresh - m*self.min_distance
            self.close_rays_thresh = int(m*avg_dist_x + c)
            # cap the rays_thresh
            if self.close_edges_thresh > self.close_edges_thresh_param.get_parameter_value().integer_value:
                self.rays_radius = self.close_edges_thresh_param.get_parameter_value().integer_value
            if self.rays_radius < self.far_rays_thresh:
                self.rays_radius = self.far_rays_thresh
        else:
            self.close_rays_thresh = self.close_rays_thresh_param.get_parameter_value().integer_value

        for i in range(-self.close_rays_thresh//2, self.close_rays_thresh//2):
            var_index = (i + self.d_1[0])
            var_index_2nd = (i + self.d_2[0])
            if 0 <= var_index < len(self.filtered_scan_msg.ranges):
                # filter the readings
                self.filtered_scan_msg.ranges[var_index] = self.d_1[1]
            if 0 <= var_index_2nd < len(self.filtered_scan_msg.ranges):
                self.filtered_scan_msg.ranges[var_index_2nd] = self.d_2[1]

    def find_theta_from_longest_ray(self):
        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if self.filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = self.filtered_scan_msg.ranges[i]
                theta = math.degrees(self.scan_msg.angle_min + self.scan_msg.angle_increment * i)
        return theta

    def get_theta_target_5(self, poss_edges:list):
        if len(poss_edges) == 0:
            return 0.0

        self.find_critical_two_edges(poss_edges)

        self.filter_scan()

        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(self.filtered_scan_msg)

        return self.find_theta_from_longest_ray()
 

    def filter_scan_cb(self, msg:LaserScan):
        self.scan_msg = msg
        smaller_angle = math.radians(-self.limit_angle)
        self.smaller_angle_index = int((smaller_angle - msg.angle_min)/msg.angle_increment)
        bigger_angle = math.radians(self.limit_angle)
        self.bigger_angle_index = int((bigger_angle - msg.angle_min)/msg.angle_increment)
        self.possible_edges = self.find_possible_edges()
        self.theta = self.get_theta_target_5(poss_edges=self.possible_edges)

    def follow_the_gap(self):
        ref_angle = 0.0
        error = (self.theta - ref_angle)
        p_controller = self.kp * error
        steering_angle = p_controller
        steering_angle = math.radians(steering_angle)
        self.vel_cmd.drive.steering_angle = steering_angle
        self.pub_vel_cmd.publish(self.vel_cmd)
        # string = ""
        # for tub in self.possible_edges:
        #     string += str(f"{tub[0]}  {tub[2]} | ")
        # if self.d_1 and self.d_2:
        #     self.get_logger().info(f"theta: {self.theta} deg  {string} || {self.d_1[0]}  {self.d_2[0]}")
        if self.d_1 and self.d_2:
            self.get_logger().info(f"theta:{self.theta:.2f} || {self.d_1[1]:.2f} * {self.d_1[2]:.2f}   {self.d_2[1]:.2f} -- {self.d_2[2]:.2f} || edges: {len(self.possible_edges)} || arc: {self.close_rays_thresh}" )

def main():
    rclpy.init()
    node = SteeringSpeedNode()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    