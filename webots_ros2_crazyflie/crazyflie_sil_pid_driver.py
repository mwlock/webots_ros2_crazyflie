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

import cffirmware as firm

class CrazyflieDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

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
        
    #     # State
    #     self.__target_twist = Twist()
    #     self.__vertical_ref = LIFT_HEIGHT
    #     self.__linear_x_integral = 0
    #     self.__linear_y_integral = 0

    #     # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('crazyflie_sil_pid_driver')
    #     self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    # def __cmd_vel_callback(self, twist):
    #     self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        ## Get measurements
        x, y, z                                     = self.__gps.getValues()
        roll_velocity, pitch_velocity, twist_yaw    = self.__gyro.getValues()
        vx, vy, vz                                  = self.__gps.getSpeedVector()
        roll, pitch, yaw                            = self.__imu.getRollPitchYaw()
                
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
        self.sensors.gyro.z = math.degrees(twist_yaw)
        
        yawDesired = 0
        forwardDesired = 0
        sidewaysDesired = 0
        
        ## Fill in Setpoints
        self.setpoint.mode.z            = firm.modeAbs
        self.setpoint.position.z        = 1.0
        self.setpoint.mode.yaw          = firm.modeVelocity
        self.setpoint.attitudeRate.yaw  = math.degrees(yawDesired)
        self.setpoint.mode.x            = firm.modeVelocity
        self.setpoint.mode.y            = firm.modeVelocity
        self.setpoint.velocity.x        = forwardDesired
        self.setpoint.velocity.y        = sidewaysDesired
        self.setpoint.velocity_body     = True
        
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
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)
        
        print("Motor powers: ", motorPower_m1, motorPower_m2, motorPower_m3, motorPower_m4)