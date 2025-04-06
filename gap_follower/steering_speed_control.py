#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import math
import copy

class SteeringSpeedNode(Node):
    def __init__(self):
        super().__init__("gap_steering_joy_node")
        self.get_logger().info("the steering node has started")
        self.scan_param = self.declare_parameter("scan_topic", "/scan")
        self.sub_scan = self.create_subscription(LaserScan, self.scan_param.get_parameter_value().string_value, self.filter_scan_cb, 10)
        self.filtered_scan_param = self.declare_parameter("filtered_scan_topic", "/filtered_scan")
        self.filtered_laser_scan_pub = self.create_publisher(LaserScan, self.filtered_scan_param.get_parameter_value().string_value, 10)
        self.drive_param = self.declare_parameter("drive_topic", "/drive")
        self.pub_vel_cmd = self.create_publisher(AckermannDriveStamped, self.drive_param.get_parameter_value().string_value, 10)
        self.tmr = self.create_timer(0.0001, self.follow_the_gap)
        self.limit_angle_param = self.declare_parameter("limit_angle", 70.0)
        self.limit_angle = self.limit_angle_param.get_parameter_value().double_value # in degrees
        self.scan_msg = LaserScan()
        self.filtered_scan_msg = LaserScan()
        self.vel_cmd = AckermannDriveStamped()
        self.obs_thresh_param = self.declare_parameter("obstacle_distance_thresh", 0.8)
        self.obs_thresh = self.obs_thresh_param.get_parameter_value().double_value     
        self.theta = 0.0 
        self.steering_angle = 0.0
        self.possible_edges = []
        self.kp_param = self.declare_parameter("kp", 1.0)
        self.kp = self.kp_param.get_parameter_value().double_value
        self.kd_param = self.declare_parameter("kd", 1.0)
        self.kd = self.kp_param.get_parameter_value().double_value
        self.prev_error = 0.0
        self.constant_speed_param = self.declare_parameter('constant_speed', 1.0)
        self.constant_speed = self.constant_speed_param.get_parameter_value().double_value
        self.dangerous_edges = []
        self.arc_length_param = self.declare_parameter('arc_length', 0.4)
        self.arc_length = self.arc_length_param.get_parameter_value().double_value
        self.min_distance_param = self.declare_parameter("min_distance", 5.0)
        self.min_distance = self.min_distance_param.get_parameter_value().double_value
        self.max_distance_param = self.declare_parameter("max_distance", 9.0)
        self.max_distance = self.max_distance_param.get_parameter_value().double_value
        self.min_vel_param = self.declare_parameter('min_vel', 0.2)
        self.min_vel = self.min_vel_param.get_parameter_value().double_value
        self.max_vel_param = self.declare_parameter('max_vel', 5.0)
        self.max_vel = self.max_vel_param.get_parameter_value().double_value
        self.linear_velocity = 0.0
        self.prev_edge = None
        self.override_steering = False
        self.activate_autonomous_vel = False
        # useless params #
        self.right_left_distance_thresh_param = self.declare_parameter('right_left_distance_thresh', 0.2)
        self.right_left_distance_thyresh = self.right_left_distance_thresh_param.get_parameter_value().double_value
        self.right_left_sides_angle_param = self.declare_parameter('right_left_sides_angle', 90.0)
        self.close_edges_thresh_param = self.declare_parameter("close_edges_thresh", 0.15)
        self.close_edges_thresh = self.close_edges_thresh_param.get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
    
    def joy_callback(self, msg:Joy):
        if msg.buttons[4] == 1:
            # self.vel_cmd.drive.speed = self.linear_velocity
            self.activate_autonomous_vel = True
        else:
            self.activate_autonomous_vel = False
    
    def find_sorted_possible_edges(self, scan_msg: LaserScan):
        ranges = scan_msg.ranges
        possible_edges = []
        for i in range(self.smaller_angle_index, self.bigger_angle_index-1):
            is_left = True
            if abs(ranges[i] - ranges[i+1]) > self.obs_thresh: # we found an edge (index, distance, angle, is_left, rays_radius)
                # we iterate from right to left
                if ranges[i] > ranges[i+1]: # that means we want to block the right block
                    is_left = False
                else:
                    is_left = True
                angle = math.degrees(scan_msg.angle_min + scan_msg.angle_increment * i)
                possible_edges.append([i, min(ranges[i], ranges[i+1]), angle, is_left, 0.0]) # keep it 0.0 as we know when problems happen
        
        # sorted ascendingly based on the distance of each edge
        possible_edges.sort(key=lambda x: x[1])
        return possible_edges
    
    def find_n_critical_edges(self, sorted_poss_edges:list, n):
        # the dangerous edges are sorted ascendingly
        if len(sorted_poss_edges) == 0:
            return []
        sorted_dangerous_edges = []
        if len(sorted_poss_edges) < n: # if the desired number of critical edges was more that the number of the critical edges
            n = len(sorted_poss_edges)
        for i in range(n):
            sorted_dangerous_edges.append(sorted_poss_edges[i])
        return sorted_dangerous_edges

    def filter_scan(self, scan_msg:LaserScan, sorted_dangerous_edges:list):
        if len(sorted_dangerous_edges) == 0:
            return scan_msg
            
        filtered_scan_msg = copy.deepcopy(scan_msg)
        # this is very important to filter the readings fromt the least significant edge to the most one
        # because we overwrite the least important dangerous edge by the important one.
        # we iterate in reversed order to move from the least dangerous edge to the most dangerous ones.
        # we can improve and decide when the car has a very far prespective it unifies all distances greater
        # than max_distance into max_distance and then go in the middle of this circle. 
        for curr_edge in reversed(sorted_dangerous_edges):

            # to make the arc,the car's front end, constant
            theta_rad = self.arc_length/curr_edge[1]
            rays_radius = int(theta_rad/scan_msg.angle_increment)
            rays_radius *= 2 

            # adding the rays thresh to the dangerous edges (index, distance, angle, is_left, rays_radius)
            curr_edge[4] = rays_radius

            from_index = -rays_radius//2
            to_index = rays_radius//2
            for i in range(from_index, to_index):
                var_index = (i + curr_edge[0])
                if 0 <= var_index < len(filtered_scan_msg.ranges):
                    filtered_scan_msg.ranges[var_index] = curr_edge[1]

        return filtered_scan_msg

    def find_theta_from_longest_ray(self, filtered_scan_msg: LaserScan):
        # check if the sides of the car are occupied
        # to continue moving if the car is close to the sides of the track
        # right_index = int((math.radians(-self.right_left_sides_angle_param.get_parameter_value().double_value) - self.scan_msg.angle_min)/self.scan_msg.angle_increment)
        # left_index = int((math.radians(self.right_left_sides_angle_param.get_parameter_value().double_value) - self.scan_msg.angle_min)/self.scan_msg.angle_increment)
        # if self.scan_msg.ranges[right_index] < self.right_left_distance_thresh or self.scan_msg.ranges[left_index] < self.right_left_distance_thresh:
        #     self.get_logger().info("continue moving --> close wall")
        #     return 0.0
        # get the longest ray
        the_longest_ray = -1.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = filtered_scan_msg.ranges[i]
                theta = math.degrees(filtered_scan_msg.angle_min + filtered_scan_msg.angle_increment * i)
        
        return theta

    def get_theta_target_5(self):
        # find the possible edges
        possible_edges = self.find_sorted_possible_edges(scan_msg=self.scan_msg)
        self.possible_edges = possible_edges

        self.dangerous_edges = self.find_n_critical_edges(possible_edges, 2)
        # filter scan message
        filtered_scan_msg = self.filter_scan(self.scan_msg, self.dangerous_edges)
        
        # publish filtered scan data
        self.filtered_laser_scan_pub.publish(filtered_scan_msg)

        return self.find_theta_from_longest_ray(filtered_scan_msg)
 
    def find_linear_vel(self):
        min_scan_ray_dist = min(self.scan_msg.ranges[self.smaller_angle_index:self.bigger_angle_index])

        # if the car cannot see any obstacles
        if len(self.possible_edges) == 0:
            # if the car is very close
            if min_scan_ray_dist < self.min_distance:  
                self.override_steering = True          
                return self.find_linear_vel_if_too_close()
            else:
                distance_x = min_scan_ray_dist
        else:
            # the distance is the distance of the closest edge
            distance_x = self.dangerous_edges[0][1]
        
        self.override_steering = False
        m = (self.max_vel - self.min_vel)/(self.max_distance - self.min_distance)
        c = self.max_vel - m*(self.max_distance)
            
        linear_vel = m*distance_x + c

        return linear_vel

    def find_linear_vel_steering_controlled(self):
        min_scan_ray_dist = min(self.scan_msg.ranges[self.smaller_angle_index:self.bigger_angle_index])

        # if the car cannot see any obstacles
        if len(self.possible_edges) == 0:
            # if the car is very close
            if min_scan_ray_dist < self.min_distance:  
                self.override_steering = True          
                return self.find_linear_vel_if_too_close()
            else:
                angle_x = abs(self.vel_cmd.drive.steering_angle)
        else:
            # the distance is the distance of the closest edge
            angle_x = abs(self.vel_cmd.drive.steering_angle)
        
        self.override_steering = False
        m = (self.max_vel - self.min_vel)/(0.0 - self.scan_msg.angle_max)
        c = self.max_vel - m*(self.scan_msg.angle_max)
            
        linear_vel = m*angle_x + c

        return linear_vel

    def find_linear_vel_if_too_close(self)->float:
        if self.prev_edge == None:
            self.get_logger().info(f"there is no prev edge")
            self.prev_edge = [0, 0.0, False, 0] # default is right

        # if was left
        if self.prev_edge[3] == True:
            self.vel_cmd.drive.steering_angle = -2.7
            self.get_logger().info(f"{self.min_distance} back.... left")
        else:
            self.vel_cmd.drive.steering_angle = 2.7
            self.get_logger().info(f"{self.min_distance} back.... right")
        
        linear_vel = -0.4
        return linear_vel

    def filter_scan_cb(self, msg:LaserScan):
        self.scan_msg = msg
        smaller_angle = math.radians(-self.limit_angle)
        self.smaller_angle_index = int((smaller_angle - msg.angle_min)/msg.angle_increment)
        bigger_angle = math.radians(self.limit_angle)
        self.bigger_angle_index = int((bigger_angle - msg.angle_min)/msg.angle_increment)
        # remember to extract first functions inside get_theta and put it here to be more clear and pass them to both functions 'theta and linear'
        self.theta = self.get_theta_target_5()
        # self.linear_velocity = self.find_linear_vel_steering_controlled()
        self.linear_velocity = self.find_linear_vel()

    def follow_the_gap(self):
        ref_angle = 0.0
        error = (self.theta - ref_angle)
        p_controller = self.kp * error
        d_controller = (self.prev_error - error) * self.kd
        steering_angle = p_controller + d_controller
        self.prev_error = error
        self.steering_angle = math.radians(steering_angle)
        if not self.override_steering:
            self.vel_cmd.drive.steering_angle = self.steering_angle 
        
        if self.activate_autonomous_vel:
            self.vel_cmd.drive.speed = self.linear_velocity
        else:
            self.vel_cmd.drive.speed = 0.0
        self.pub_vel_cmd.publish(self.vel_cmd)
        self.get_logger().info(f"θ:{self.theta:.2f} || v: {self.linear_velocity:.2f} || e: {len(self.possible_edges)} || d_e: {self.dangerous_edges} || {len(self.dangerous_edges)}" )

def main():
    rclpy.init()
    node = SteeringSpeedNode()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()