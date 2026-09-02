#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import sys
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PurePursuit:
    def __init__(self, robot_name, path_file):
        rospy.init_node(f'pure_pursuit_tracker_{robot_name}', anonymous=True)
        
        self.robot_name = robot_name
        self.paused = False

        self.speed_boosted = False
        self.speed_boost_time = None

        
        self.lookahead_distance = 0.2
        self.linear_speed = 0.2
        self.min_speed = 0.05
        self.reach_threshold = 0.1
        

        self.path = []  # (x, y, yaw)
        self.current_pose = None
        self.current_yaw = 0.0
        self.actual_path = Path()
        self.actual_path.header.frame_id = f'{robot_name}/odom'
        self.last_actual_path_point = None

        self.load_path(path_file)

        self.cmd_pub = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
        # Latching keeps the complete CSV path visible even when RViz starts later.
        self.planned_path_pub = rospy.Publisher(
            f'/{robot_name}/planned_path', Path, queue_size=1, latch=True)
        self.actual_path_pub = rospy.Publisher(
            f'/{robot_name}/actual_path', Path, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/{robot_name}/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)
        self.publish_planned_path()

    def load_path(self, filepath):
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                self.path.append((float(row[0]), float(row[1]), math.radians(float(row[2]))))

        if not self.path:
            raise ValueError(f'Path file is empty: {filepath}')

    def publish_planned_path(self):
        """Publish the complete reference path loaded from the CSV file."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'world'

        for x, y, yaw in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x, pose.pose.orientation.y, \
                pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(0, 0, yaw)
            path_msg.poses.append(pose)

        self.planned_path_pub.publish(path_msg)
        rospy.loginfo("Published %d planned path points on /%s/planned_path",
                      len(path_msg.poses), self.robot_name)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_pose = (pos.x, pos.y)

        ori = msg.pose.pose.orientation
        orientation_q = [ori.x, ori.y, ori.z, ori.w]
        (_, _, yaw) = euler_from_quaternion(orientation_q)
        self.current_yaw = yaw

        # Store one point every 3 cm, avoiding an unnecessarily large Path at
        # the 100 Hz odometry update rate.
        if (self.last_actual_path_point is None or
                math.hypot(pos.x - self.last_actual_path_point[0],
                           pos.y - self.last_actual_path_point[1]) >= 0.03):
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.actual_path.header.frame_id
            pose.pose = msg.pose.pose
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path.poses.append(pose)
            self.last_actual_path_point = (pos.x, pos.y)
            self.actual_path_pub.publish(self.actual_path)

    def find_lookahead_point(self, start_index=0):
        if not self.current_pose:
            return None
        x, y = self.current_pose
        for i in range(start_index, len(self.path)):
            px, py, _ = self.path[i]
            dx = px - x
            dy = py - y
            distance = math.hypot(dx, dy)
            if distance >= self.lookahead_distance:
                return (px, py, i)
        return None

    def compute_control_command(self, target):
        x, y = self.current_pose
        tx, ty, _ = target
        dx = tx - x
        dy = ty - y

        Ld = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        alpha_deg = math.degrees(alpha)

        if Ld > 0.01:
            curvature = 2 * math.sin(alpha) / Ld
        else:
            curvature = 0.0

        w = -4 * self.linear_speed * curvature
        w = max(min(w, 0.8), -0.8)

        v = self.linear_speed if abs(alpha_deg) < 30 else self.min_speed

        rospy.loginfo(f"[DEBUG] yaw = {math.degrees(self.current_yaw):.1f}°, target_angle = {math.degrees(target_angle):.1f}°, alpha = {alpha_deg:.1f}°")
        return v, w, dx, dy, alpha, Ld

    def rotate_to_target_yaw(self, target_yaw_rad):
        yaw_tolerance = math.radians(5.0)  # 允许误差3度
        angular_speed = -0.3

        rospy.loginfo(f"🎯 正在调整朝向至 {math.degrees(target_yaw_rad):.1f}° ...")

        while not rospy.is_shutdown():
            yaw_error = target_yaw_rad - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            if abs(yaw_error) < yaw_tolerance:
                break

            cmd = Twist()
            cmd.angular.z = angular_speed if yaw_error > 0 else -angular_speed
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        rospy.loginfo("✅ 朝向调整完成")
        self.cmd_pub.publish(Twist())

    def run(self):
        final_target = (self.path[-1][0], self.path[-1][1])
        current_index = 0

        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue
                
                


            # ===== robot_2 加速控制逻辑 =====
            if self.robot_name == "robot_2":
                px, py = self.current_pose

                if (not self.speed_boosted and 
                    abs(px - 5.1) < 0.05 and abs(py - 6.43) < 0.05):
                    self.linear_speed = 0.3
                    self.speed_boosted = True
                    self.speed_boost_time = rospy.Time.now()
                    rospy.loginfo("🚀 robot_2 进入加速区域，速度提升至 0.3 m/s")

                if self.speed_boosted:
                    elapsed = (rospy.Time.now() - self.speed_boost_time).to_sec()
                    if elapsed >= 3.0:
                        self.linear_speed = 0.2
                        
                        
            # ===== robot_3 加速控制逻辑 =====
            if self.robot_name == "robot_3":
                px, py = self.current_pose

                if (not self.speed_boosted and 
                    abs(px - 5.27) < 0.05 and abs(py - 7.18) < 0.05):
                    self.linear_speed = 0.4
                    self.speed_boosted = True
                    self.speed_boost_time = rospy.Time.now()
                    rospy.loginfo("🚀 robot_3 进入加速区域，速度提升至 0.4 m/s")

                if self.speed_boosted:
                    elapsed = (rospy.Time.now() - self.speed_boost_time).to_sec()
                    if elapsed >= 6.5:
                        self.linear_speed = 0.2


            target_info = self.find_lookahead_point(current_index)

            if target_info is None:
                dx = final_target[0] - self.current_pose[0]
                dy = final_target[1] - self.current_pose[1]
                distance = math.hypot(dx, dy)

                if distance < self.reach_threshold:
                    rospy.loginfo("🎯 已到达路径终点，停车")
                    self.cmd_pub.publish(Twist())
                    # ✅ 终点后调整为朝上方向
                    self.rotate_to_target_yaw(0)   #向下是-1.57
                    break
                else:
                    rospy.loginfo("⚠️ 使用终点作为目标继续追踪")
                    target_info = (final_target[0], final_target[1], len(self.path)-1)
            else:
                current_index = max(current_index, target_info[2])
                if current_index + 1 < len(self.path):
                    target_info = (self.path[current_index+1][0], self.path[current_index+1][1], current_index+1)

            v, w, dx, dy, alpha, distance = self.compute_control_command(target_info)



            x, y = self.current_pose
            rospy.loginfo(f"\n--- Pure Pursuit ---"
                          f"\n当前位置: x = {x:.2f}, y = {y:.2f}, yaw = {math.degrees(self.current_yaw):.1f}°"
                          f"\n目标点:   x = {target_info[0]:.2f}, y = {target_info[1]:.2f}"
                          f"\n误差向量: dx = {dx:.2f}, dy = {dy:.2f}"
                          f"\n目标角偏差 alpha = {math.degrees(alpha):.1f}°"
                          f"\n前瞻距离: {distance:.2f} m"
                          f"\n控制指令: 线速度 v = {v:.2f}, 角速度 w = {w:.2f}\n")

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            
            
            # ====== robot_2 指定点暂停 1 秒 ======      zz的路径需要
            if self.robot_name == "robot_1" and not self.paused and current_index > 1:
                px, py = self.current_pose
                dx = abs(px - 0.70)
                dy = abs(py + 0.51)
                if dx < 0.03 and dy < 0.03:
                    rospy.loginfo("🕒 robot_2 到达指定点 (1.45, 1.64)，暂停 1 秒")
                    self.cmd_pub.publish(Twist())  # 停止运动
                    rospy.sleep(12.0)
                    self.paused = True     
                    
            # ====== robot_2 指定点暂停 1 秒 ======
            if self.robot_name == "robot_2" and not self.paused and current_index > 1:
                px, py = self.current_pose
                dx = abs(px - 1.27)
                dy = abs(py - 1.6)
                if dx < 0.03 and dy < 0.03:
                    rospy.loginfo("🕒 robot_2 到达指定点 (1.45, 1.64)，暂停 1 秒")
                    self.cmd_pub.publish(Twist())  # 停止运动
                    rospy.sleep(4.0)
                    self.paused = True


            # ====== robot_3 指定点暂停 1 秒 ======
            if self.robot_name == "robot_3" and not self.paused and current_index > 1:
                px, py = self.current_pose
                dx = abs(px - 2)
                dy = abs(py - 1.12)
                if dx < 0.1 and dy < 0.1:
                    rospy.loginfo("🕒 robot_3 到达指定点 (2,1.12)，暂停 1 秒")
                    self.cmd_pub.publish(Twist())  # 停止运动
                    rospy.sleep(6.0)
                    self.paused = True


            self.rate.sleep()


        rospy.loginfo("✅ 任务完成")
        self.cmd_pub.publish(Twist())


if __name__ == '__main__':
    try:
        if len(sys.argv) < 3:
            print("Usage: pure_pursuit.py <robot_name> <path_file>")
            sys.exit(1)
        robot_name = sys.argv[1]
        path_file = sys.argv[2]
        controller = PurePursuit(robot_name, path_file)
        controller.run()
    except rospy.ROSInterruptException:
        pass
