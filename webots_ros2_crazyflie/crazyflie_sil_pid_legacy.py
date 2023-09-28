# Copyright 2023 Matthew Lock.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bitcraze Crazyflie driver."""

import rclpy
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

import cffirmware as firm

class CrazyflieDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.robot_name = self.__robot.getName()

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial_unit')

        # Propellers
        self.m1_motor = self.__robot.getDevice('m1_motor')
        self.m2_motor = self.__robot.getDevice('m2_motor')
        self.m3_motor = self.__robot.getDevice('m3_motor')
        self.m4_motor = self.__robot.getDevice('m4_motor')
        
        self.__propellers = [
            self.m1_motor,
            self.m2_motor,
            self.m3_motor,
            self.m4_motor
        ]
        
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)
            
        # Firmware initialization
        firm.controllerPidInit()
        self.state      = firm.state_t()
        self.sensors    = firm.sensorData_t()
        self.setpoint   = firm.setpoint_t()
        self.control    = firm.control_t()
        self.tick       = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        
        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('crazyflie_sil_pid_legacy')
        
        self.pose_publisher = self.__node.create_publisher(
            PoseStamped,
            f"{self.robot_name}/pose",
            1
        )
        self.twist_publisher = self.__node.create_publisher(
            TwistStamped,
            f"{self.robot_name}/twist",
            1
        )
        self.cmd_vel_subscription = self.__node.create_subscription(
            Twist,
            f"{self.robot_name}/cmd_vel",
            self.cmd_vel_callback,
            1
        )
        
        # States
        self.go_to_start_bool   = True
        self.hovering_time      = 0
        self.start_mission_time = 0
        self.last_time          = self.__robot.getTime()
        self.__node.get_logger().info(f"{str(self.__timestep)}")

        #roslog info
        self.logger = self.__node.get_logger()
        self.logger.info(f"Robot {self.robot_name} initialized")
        
        # Initial position
        self.des_x = 0.0
        self.des_y = 0.0
        self.des_z = 1.0
        self.yaw_desired_degrees = 0.0
        self.enable_hovering()
        
        self.setpoint.velocity_body     = False
        
        self.logger.info(f"Asserting control mode")
        assert self.control.controlMode == firm.controlModeLegacy
        
        
    def cmd_vel_callback(self, msg:Twist):
        
        # Roll, pitch, yawrate, thrust
        roll    = msg.linear.x
        pitch   = msg.linear.y
        yawrate = msg.angular.z
        thrust  = min(max(msg.linear.z,0), 6000)
        
        self.setpoint.attitude.roll     = roll
        self.setpoint.attitude.pitch    = pitch
        self.setpoint.attitudeRate.yaw  = yawrate
        self.setpoint.thrust            = thrust
        
    def enable_hovering(self):
        
        ## Fill in Setpoints
        self.setpoint.mode.z            =   firm.modeAbs
        self.setpoint.mode.y            =   firm.modeVelocity
        self.setpoint.mode.x            =   firm.modeVelocity
        self.setpoint.position.z        = self.des_z
        
        rad_conv = 8/0.5        
        yaw_desired_rad = math.radians(self.yaw_desired_degrees*rad_conv)
        self.setpoint.velocity.x = self.des_x
        self.setpoint.velocity.y = self.des_y
        self.setpoint.attitudeRate.yaw = math.degrees(yaw_desired_rad)
        
    def enable_legacy_control(self):
        
        self.setpoint.mode.z            =   firm.modeDisable    # Ensure manual position control by ensuring thrust is used (see firmware - controller_pid.c)
        self.setpoint.mode.y            =   firm.modeDisable    # Ensure attitudeDesired.roll/roll is set (see firmware - controller_pid.c)
        self.setpoint.mode.x            =   firm.modeDisable    # Ensure attitudeDesired.roll/roll is set (see firmware - controller_pid.c)
        self.setpoint.mode.yaw          =   firm.modeVelocity
        
    def enable_position_control(self):
        
        self.setpoint.mode.z            =   firm.modeAbs
        self.setpoint.mode.y            =   firm.modeVelocity 
        self.setpoint.mode.x            =   firm.modeVelocity
        
        self.setpoint.velocity.x = 0.0
        self.setpoint.velocity.y = 0.0
        self.setpoint.position.z = 1
    
        
    def go_to_pose(self):
               
        ## Firmware PID bindings
        firm.controllerPid(
            self.control, 
            self.setpoint,
            self.sensors,
            self.state,
            self.tick
        )
        
        ## Motor commands
        cmd_roll    =   math.radians(self.control.roll)
        cmd_pitch   =   math.radians(self.control.pitch)
        cmd_yaw     = - math.radians(self.control.yaw)
        cmd_thrust  =   self.control.thrust
        
        ## Motor mixing
        motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
        
        scaling = 1000 ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(- motorPower_m1/scaling)
        self.m2_motor.setVelocity(  motorPower_m2/scaling)
        self.m3_motor.setVelocity(- motorPower_m3/scaling)
        self.m4_motor.setVelocity(  motorPower_m4/scaling)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        ## Get measurements
        x, y, z                                         = self.__gps.getValues()
        vx, vy, vz                                      = self.__gps.getSpeedVector()
        roll, pitch, yaw                                = self.__imu.getRollPitchYaw()
        roll_velocity, pitch_velocity, yaw_velocity     = self.__gyro.getValues()
                
        if math.isnan(vx):
            return
        
        ## Put measurement in state estimate
        self.state.attitude.roll    =   math.degrees(roll)
        self.state.attitude.pitch   = - math.degrees(pitch)
        self.state.attitude.yaw     =   math.degrees(yaw)
        self.state.position.x       =   x
        self.state.position.y       =   y
        self.state.position.z       =   z
        self.state.velocity.x       =   vx
        self.state.velocity.y       =   vy
        self.state.velocity.z       =   vz
            
        # Put gyro in sensor data
        self.sensors.gyro.x = math.degrees(roll_velocity)
        self.sensors.gyro.y = math.degrees(pitch_velocity)
        self.sensors.gyro.z = math.degrees(yaw_velocity)
        
        # publish pose
        pose = PoseStamped()
        pose.header.stamp = self.__node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = roll
        pose.pose.orientation.y = pitch
        pose.pose.orientation.z = yaw
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)
        
        twist = TwistStamped()
        twist.header.stamp = self.__node.get_clock().now().to_msg()
        twist.header.frame_id = 'map'
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = roll_velocity
        twist.twist.angular.y = pitch_velocity
        twist.twist.angular.z = yaw_velocity
        self.twist_publisher.publish(twist)
        
        self.go_to_pose()