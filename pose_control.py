#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


class ControlPose():
    def __init__(self):
        self.subscriber = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.control_pub = rospy.Publisher("/complete_control", Bool, queue_size=1)
        #比例ゲイン
        self.Kp = 0.8
        #現在の角度[rad]
        self.angle = 0.0
        #目標角度[rad]
        self.angle_ref = 0.0
        #前回の偏差[rad]
        self.prev_error = 0.0

        #制御完了フラグ
        self.complete_control = False
    

    #クオータニオンからオイラー角への変換
    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    #IMUのトピックをコールバックして現在の角度を取得
    def imu_callback(self, msg):
    
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        #クオータニオンからオイラー角に変換
        euler = self.quaternion_to_euler(Quaternion(x, y, z, w))

        self.angle = euler.z

    #制御量の計算および出力
    def control_angle(self, angle):
        self.angle_ref = angle
        #偏差の計算[rad]
        error = self.angle_ref - self.angle
        #制御量の計算
        twist_z = self.Kp * error

        #/cmd_velをパブリッシュ
        twist = Twist()
        twist.angular.z = twist_z
        self.cmd_pub.publish(twist)
        print(twist_z)
        
        #偏差がしきい値の範囲だったら制御完了フラグをTrueにする
        if error <= 0.05 and error >= -0.05:
            self.complete_control = True
        else :
            self.complete_control = False
        self.control_pub.publish(self.complete_control)

if __name__ == "__main__":
    rospy.init_node("pose_control")

    control_pose = ControlPose()
 
    while not rospy.is_shutdown():
        control_pose.control_angle(0.0)

    rospy.spin() 