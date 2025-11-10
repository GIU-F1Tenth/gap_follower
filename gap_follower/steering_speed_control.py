#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
import math
import copy
import numpy as np
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data

class SteeringSpeedNode(Node):
    def __init__(self):
        super().__init__("gap_steering_diffdrive_node")
        self.get_logger().info("the steering node has started (diff-drive)")

        # Topics & publishers
        self.scan_param = self.declare_parameter("scan_topic", "/scan")
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_param.get_parameter_value().string_value,
            self.filter_scan_cb,
            qos_profile_sensor_data
        )

        self.filtered_scan_param = self.declare_parameter("filtered_scan_topic", "/filtered_scan")
        self.filtered_laser_scan_pub = self.create_publisher(
            LaserScan,
            self.filtered_scan_param.get_parameter_value().string_value,
            10
        )

        # For differential drive we publish Twist (default topic /cmd_vel)
        self.drive_param = self.declare_parameter("drive_topic", "/cmd_vel")
        self.pub_vel_cmd = self.create_publisher(Twist, self.drive_param.get_parameter_value().string_value, 10)

        # Timer (you used 0.001 previously, which is 1kHz; keep but be careful with CPU)
        self.tmr = self.create_timer(0.001, self.follow_the_gap)

        # Parameters (keep same defaults as before)
        self.limit_angle_param = self.declare_parameter("limit_angle", 70.0)
        self.limit_angle = self.limit_angle_param.get_parameter_value().double_value # degrees

        self.scan_msg = LaserScan()
        self.filtered_scan_msg = LaserScan()

        # Use Twist instead of AckermannDriveStamped
        self.vel_cmd = Twist()

        self.number_of_critical_edges_param = self.declare_parameter("number_of_critical_edges", 2)
        self.number_of_critical_edges = self.number_of_critical_edges_param.get_parameter_value().integer_value

        self.obs_thresh_param = self.declare_parameter("obstacle_distance_thresh", 0.8)
        self.obs_thresh = self.obs_thresh_param.get_parameter_value().double_value

        self.theta = 0.0 
        self.steering_angle = 0.0
        self.possible_edges = []

        self.kp_param = self.declare_parameter("kp", 1.0)
        self.kp = self.kp_param.get_parameter_value().double_value
        self.kd_param = self.declare_parameter("kd", 1.0)
        self.kd = self.kd_param.get_parameter_value().double_value
        self.prev_error = 0.0
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

        self.k_sigmoid_steering_param = self.declare_parameter('k_sigmoid_steering', 8.0)
        self.k_sigmoid_steering = self.k_sigmoid_steering_param.get_parameter_value().double_value
        self.k_sigmoid_linear_vel_param = self.declare_parameter('k_sigmoid_linear_vel', 0.5)
        self.k_sigmoid_linear_vel = self.k_sigmoid_linear_vel_param.get_parameter_value().double_value

        self.linear_velocity = 0.0
        self.prev_edge = None
        self.override_steering = False
        self.activate_autonomous_vel = True
        self.is_active = True   
        self.stop = False 

        # Subs to toggle
        self.pause_sub = self.create_subscription(Bool, "/pause", self.toggle_stop, 10)
        self.gap_follower_toggle_sub = self.create_subscription(Bool, "/gap_follower_toggle", self.toggle_algo_cb, 10)

    def toggle_stop(self, msg:Bool):
        self.stop = msg.data
        if self.stop:
            self.get_logger().info("Stopping the car")
            self.stop = True
        else:
            self.get_logger().info("Resuming the car")
            self.stop = False

    def toggle_algo_cb(self, msg:Bool):
        self.is_active = msg.data

    def find_sorted_possible_edges(self, scan_msg: LaserScan):
        ranges = scan_msg.ranges
        possible_edges = []
        for i in range(self.smaller_angle_index, self.bigger_angle_index-1):
            if i+1 >= len(ranges):
                break
            if abs(ranges[i] - ranges[i+1]) > self.obs_thresh:
                # we iterate from right to left
                if ranges[i] > ranges[i+1]:
                    is_left = False
                else:
                    is_left = True
                angle = math.degrees(scan_msg.angle_min + scan_msg.angle_increment * i)
                possible_edges.append([i, min(ranges[i], ranges[i+1]), angle, is_left, 0.0])
        possible_edges.sort(key=lambda x: x[1])
        return possible_edges

    def find_n_critical_edges(self, sorted_poss_edges:list, n):
        if len(sorted_poss_edges) == 0:
            return []
        if len(sorted_poss_edges) < n:
            n = len(sorted_poss_edges)
        return sorted_poss_edges[:n]

    def filter_scan(self, scan_msg:LaserScan, sorted_dangerous_edges:list):
        if len(sorted_dangerous_edges) == 0:
            return scan_msg
        filtered_scan_msg = copy.deepcopy(scan_msg)
        for curr_edge in reversed(sorted_dangerous_edges):
            if curr_edge[1] <= 0.0:
                continue
            theta_rad = self.arc_length/curr_edge[1]
            rays_radius = int(theta_rad/scan_msg.angle_increment) if scan_msg.angle_increment != 0.0 else 0
            rays_radius *= 2
            curr_edge[4] = rays_radius
            from_index = -rays_radius//2
            to_index = rays_radius//2
            for i in range(from_index, to_index):
                var_index = (i + curr_edge[0])
                if 0 <= var_index < len(filtered_scan_msg.ranges):
                    filtered_scan_msg.ranges[var_index] = min(curr_edge[1], scan_msg.ranges[var_index])
        return filtered_scan_msg

    def find_theta_from_longest_ray(self, filtered_scan_msg: LaserScan):
        the_longest_ray = -1.0
        theta = 0.0
        for i in range(self.smaller_angle_index, self.bigger_angle_index):
            if filtered_scan_msg.ranges[i] > the_longest_ray:
                the_longest_ray = filtered_scan_msg.ranges[i]
                theta = math.degrees(filtered_scan_msg.angle_min + filtered_scan_msg.angle_increment * i)
        self.get_logger().info(f"theta = {theta:.2f} deg, longest ray = {the_longest_ray:.2f}")
        return theta

    def get_theta_target(self): # degrees
        possible_edges = self.find_sorted_possible_edges(scan_msg=self.scan_msg)
        self.possible_edges = possible_edges
        self.dangerous_edges = self.find_n_critical_edges(possible_edges, self.number_of_critical_edges)
        filtered_scan_msg = self.filter_scan(self.scan_msg, self.dangerous_edges)
        self.filtered_laser_scan_pub.publish(filtered_scan_msg)
        return self.find_theta_from_longest_ray(filtered_scan_msg)

    def find_linear_vel_if_too_close(self)->float:
        if self.prev_edge == None:
            self.get_logger().info("there is no prev edge")
            self.prev_edge = [0, 0.0, False, 0]
        if self.prev_edge[3] == True:
            # if previous edge was left, turn right (set angular.z negative or positive depending on your robot convention)
            self.vel_cmd.angular.z = -2.7
            self.get_logger().info(f"{self.min_distance} back.... left")
        else:
            self.vel_cmd.angular.z = 2.7
            self.get_logger().info(f"{self.min_distance} back.... right")
        linear_vel = -0.4
        return linear_vel

    def find_linear_vel_steering_controlled_linearly(self):
        arr = self.scan_msg.ranges[self.smaller_angle_index:self.bigger_angle_index]
        if not arr:
            return self.min_vel
        min_scan_ray_dist = min(arr)
        if len(self.possible_edges) == 0:
            if min_scan_ray_dist < self.min_distance:
                self.override_steering = True
                return self.find_linear_vel_if_too_close()
            else:
                angle_x = abs(self.theta)
        else:
            angle_x = abs(self.theta)

        self.override_steering = False
        m = (self.max_vel - self.min_vel)/(0.0 - self.limit_angle)
        c = self.max_vel - m*(0.0)
        linear_vel = m*angle_x + c
        linear_vel = max(self.min_vel, min(self.max_vel, linear_vel))
        return linear_vel

    def find_linear_vel_steering_controlled_sigmoidally(self):
        arr = self.scan_msg.ranges[self.smaller_angle_index:self.bigger_angle_index]
        if not arr:
            return self.min_vel
        min_scan_ray_dist = min(arr)
        if len(self.possible_edges) == 0:
            if min_scan_ray_dist < self.min_distance:
                self.override_steering = True
                return self.find_linear_vel_if_too_close()
            else:
                angle_x = abs(self.theta)
        else:
            angle_x = abs(self.theta)

        self.override_steering = False

        def compute_c_sigmoid(v_min, v_max, k):
            return -1 * (1 / k) * np.log((v_max - v_max * 0.999) / (v_max * 0.999 - v_min))

        k = self.k_sigmoid_steering
        c = compute_c_sigmoid(self.min_vel, self.max_vel, k)
        vel = self.min_vel + (self.max_vel - self.min_vel) / (1 + np.exp(k * (math.radians(angle_x) - c)))
        vel = max(self.min_vel, min(self.max_vel, vel))
        return vel

    def find_linear_vel_front_ray_controlled_sigmoidally(self):
        ray_x = 0.0
        arr = self.scan_msg.ranges[self.smaller_angle_index:self.bigger_angle_index]
        if not arr:
            return self.min_vel
        min_scan_ray_dist = min(arr)
        if len(self.possible_edges) == 0:
            if min_scan_ray_dist < self.min_distance:
                self.override_steering = True
                return self.find_linear_vel_if_too_close()
            else:
                if self.scan_msg.angle_increment != 0.0:
                    ray_x = self.scan_msg.ranges[int(-self.scan_msg.angle_min / self.scan_msg.angle_increment)]
        else:
            if self.scan_msg.angle_increment != 0.0:
                ray_x = self.scan_msg.ranges[int(-self.scan_msg.angle_min / self.scan_msg.angle_increment)]

        ray_x = max(self.min_distance, min(self.max_distance, ray_x))
        self.override_steering = False

        def compute_c_sigmoid(v_min, v_max, k, max_ray_x=10):
            return (1 / k) * np.log(((v_max - v_min) / (v_max * 0.9 - v_min))-1) + max_ray_x

        k = self.k_sigmoid_linear_vel
        c = compute_c_sigmoid(self.min_vel, self.max_vel, k, self.max_distance)
        vel = self.min_vel + (self.max_vel - self.min_vel) / (1 + np.exp(k * (-ray_x + c)))
        vel = max(self.min_vel, min(self.max_vel, vel))
        return vel

    def too_close_handler(self):
        # implement custom behavior if required
        pass

    # def filter_scan_cb(self, msg:LaserScan):
    #     # copy latest scan
    #     self.scan_msg = copy.deepcopy(msg)
    #     # remove zeros and infs (you already did this)
    #     self.scan_msg.ranges = [
    #         r if (r != 0.0 and not math.isinf(r)) else msg.range_max
    #         for r in msg.ranges
    #     ]
    #     smaller_angle = math.radians(-self.limit_angle)
    #     self.smaller_angle_index = int((smaller_angle - msg.angle_min)/msg.angle_increment) if msg.angle_increment != 0.0 else 0
    #     bigger_angle = math.radians(self.limit_angle)
    #     self.bigger_angle_index = int((bigger_angle - msg.angle_min)/msg.angle_increment) if msg.angle_increment != 0.0 else len(self.scan_msg.ranges)-1

    #     # compute desired heading (degrees)
    #     self.theta = self.get_theta_target()

    #     # compute linear velocity (take min of the two sigmoid methods like before)
    #     v1 = self.find_linear_vel_steering_controlled_sigmoidally()
    #     v2 = self.find_linear_vel_front_ray_controlled_sigmoidally()
    #     self.linear_velocity = min(v1, v2)

    def filter_scan_cb(self, msg: LaserScan):
        # Copy the scan safely
        self.scan_msg = copy.deepcopy(msg)

        # Replace invalid ranges (0 or inf) with max range
        self.scan_msg.ranges = [
            r if (r != 0.0 and not math.isinf(r)) else msg.range_max
            for r in msg.ranges
        ]

        # --- 🔄 REFORMAT SCAN FROM [0, 360) → [-180, 180) ---
        num_readings = len(self.scan_msg.ranges)
        half = num_readings // 2

        # Rotate ranges by 180 degrees (π radians)
        self.scan_msg.ranges = self.scan_msg.ranges[half:] + self.scan_msg.ranges[:half]

        # Update angle metadata accordingly
        self.scan_msg.angle_min = -math.pi
        self.scan_msg.angle_max = math.pi
        self.scan_msg.angle_increment = (2 * math.pi) / num_readings

        # --- Compute smaller/larger angle window for processing ---
        smaller_angle = math.radians(-self.limit_angle)
        bigger_angle = math.radians(self.limit_angle)

        self.smaller_angle_index = int(
            (smaller_angle - self.scan_msg.angle_min) / self.scan_msg.angle_increment
        )
        self.bigger_angle_index = int(
            (bigger_angle - self.scan_msg.angle_min) / self.scan_msg.angle_increment
        )
        # --- Compute desired heading (degrees) ---
        self.theta = self.get_theta_target()

        # --- Compute linear velocity ---
        v1 = self.find_linear_vel_steering_controlled_sigmoidally()
        v2 = self.find_linear_vel_front_ray_controlled_sigmoidally()
        self.linear_velocity = min(v1, v2)


    def follow_the_gap(self):
        ref_angle = 0.0
        error = (self.theta - ref_angle)
        p_controller = self.kp * error
        d_controller = (error - self.prev_error) * self.kd
        steering_angle = p_controller + d_controller
        self.prev_error = error

        # steering_angle was in degrees from your PID; convert to radians
        self.steering_angle = math.radians(steering_angle)

        if not self.override_steering:
            # map steering to angular.z for diff-drive
            self.vel_cmd.angular.z = self.steering_angle

        if self.activate_autonomous_vel and self.is_active and not self.stop:
            self.vel_cmd.linear.x = self.linear_velocity
            self.pub_vel_cmd.publish(self.vel_cmd)
            # debug log (uncomment for more info)
            # self.get_logger().info(f"θ:{math.radians(self.theta):.2f} || v:{self.linear_velocity:.2f} || e:{len(self.possible_edges)} || de:{len(self.dangerous_edges)}")

def main():
    rclpy.init()
    node = SteeringSpeedNode()
    try:
        rclpy.spin(node=node)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
