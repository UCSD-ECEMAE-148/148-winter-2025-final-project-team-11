#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import math
import sys
import yaml
import os
from rcl_interfaces.msg import SetParametersResult

LED_TOPIC_NAME = '/led'
AREA_TOPIC_NAME = '/area'

class LaneGuidanceNode(Node):
    def __init__(self, config_file=None):
        super().__init__('lane_guidance_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        # Load config if provided
        if config_file and os.path.exists(config_file):
            self.get_logger().info(f'Loading config from: {config_file}')
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                if config and '/**' in config and 'ros__parameters' in config['/**']:
                    config = config['/**']['ros__parameters']
                
            # Extract parameters from config
            Kp_steering = config.get('Kp_steering', 1.0)
            Ki_steering = config.get('Ki_steering', 0.0)
            Kd_steering = config.get('Kd_steering', 0.0)
            max_throttle = config.get('max_throttle', 0.4)
            min_throttle = config.get('min_throttle', 0.1)
            steering_offset = config.get('steering_offset', -0.5)
            Kp_throttle = config.get('Kp_throttle', 1.0)
            Ki_throttle = config.get('Ki_throttle', 1.0)
            Kd_throttle = config.get('Kd_throttle', 1.0)
        else:
            # Default values
            Kp_steering = 1.0
            Ki_steering = 0.0
            Kd_steering = 0.0
            max_throttle = 0.4
            min_throttle = 0.1
            steering_offset = -0.5
            Kp_throttle = 1
            Ki_throttle = 1
            Kd_throttle = 1
            
        # Create subscribers
        self.centroid_sub = self.create_subscription(Float32, '/centroid', self.centroid_callback, 10 )

        self.led = self.create_publisher(Bool, LED_TOPIC_NAME, 10 )

        self.area = self.create_subscription(Float32, AREA_TOPIC_NAME, self.speed_callback, 10 )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare parameters
        self.declare_parameter('Kp_steering', Kp_steering)
        self.declare_parameter('Ki_steering', Ki_steering)
        self.declare_parameter('Kd_steering', Kd_steering)
        self.declare_parameter('max_throttle', max_throttle)
        self.declare_parameter('min_throttle', min_throttle)
        self.declare_parameter('steering_offset', steering_offset)
        self.declare_parameter('Kp_throttle', Kp_throttle)
        self.declare_parameter('Ki_throttle', Ki_throttle)
        self.declare_parameter('Kd_throttle', Kd_throttle)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Initialize variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        
        # Light setup
        self.speed_limit_count = 0
        led_init= Bool()
        led_init.data = False
        self.led.publish(led_init)
        
        # PID Speed
        self.speed = 0
        self.last_speed_error = 0
        self.speed_integral = 0.0
        self.last_speed_calc_time = self.get_clock().now()
        
        self.get_logger().info(
            f'\nKp_steering: {Kp_steering}'
            f'\nKi_steering: {Ki_steering}'
            f'\nKd_steering: {Kd_steering}'
            f'\nmax_throttle: {max_throttle}'
            f'\nmin_throttle: {min_throttle}'
        )
    
    def parameters_callback(self, params):
        """Handle parameter updates"""
        self.get_logger().info(f'Received parameter update request: {[p.name for p in params]}')
        
        try:
            for param in params:
                self.get_logger().info(f'Processing parameter: {param.name} = {param.value}')
                
                # Reset control variables based on parameter changes
                if param.name == 'Ki_steering':
                    self.integral = 0.0  # Reset integral when Ki changes
                elif param.name == 'Kd_steering':
                    self.last_error = 0.0  # Reset derivative when Kd changes
                
            self.get_logger().info('Parameters updated successfully')
            return SetParametersResult(successful=True)
            
        except Exception as e:
            self.get_logger().error(f'Error updating parameters: {e}')
            return SetParametersResult(successful=False, reason=str(e))
    
    def centroid_callback(self, msg):
        # Get current parameters
        Kp = self.get_parameter('Kp_steering').value
        Ki = self.get_parameter('Ki_steering').value
        Kd = self.get_parameter('Kd_steering').value
        max_throttle = self.get_parameter('max_throttle').value
        min_throttle = self.get_parameter('min_throttle').value
        
        # Get error from centroid
        error = msg.data
        
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        
        # PID calculations
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        # Calculate steering command using PID
        steering = Kp * error + Ki * self.integral + Kd * derivative

        steering += self.get_parameter('steering_offset').value
        
        # Limit steering to [-1, 1]
        steering = max(-1.0, min(1.0, steering))
        
        # Calculate throttle - reduce speed when turning
        # throttle = max_throttle - (abs(steering) * (max_throttle - min_throttle))
        
        # Create and publish Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = float(self.speed)
        cmd_vel.angular.z = float(steering)  # Negative because positive error means turn right
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Update state variables
        self.last_error = error
        self.last_time = current_time
        
        self.get_logger().info(f'Error: {error:.3f}, Steering: {steering:.3f}, Throttle: {self.speed:.3f}')

    def speed_callback(self, msg):
        area = msg.data
        kP = self.get_parameter('Kp_throttle').value
        kI = self.get_parameter('Ki_throttle').value
        kD = self.get_parameter('Kd_throttle').value
        power_integral_cap = 0.1
        theTargetArea = 500
        
        integral_cap = (power_integral_cap / kI) if kI != 0 else 0
        theError = theTargetArea - area

        self.speed_integral = self.speed_integral + theError
        self.speed_integral = min(integral_cap, max(-integral_cap, self.speed_integral))
        
        derivativeCalc = (theError - self.last_speed_error) / ((self.get_clock().now() - self.last_speed_calc_time).nanoseconds / 1e6) # This is in millis
        self.last_speed_error = theError
        self.last_speed_calc_time = self.get_clock().now()
        
        if (area < 650): # Car is farther away
            self.speed = 0.1 + (theError * kP) + (self.speed_integral * kI) + (derivativeCalc * kD) 
        else: # Car is too close!! 
            self.speed = 0
            led_cmd_off = Bool()
            led_cmd_off.data = False
            self.led.publish(led_cmd_off)
        
        if (self.speed > 0.2): # Set the speed limit
            self.speed_limit_count = self.speed_limit_count + 1
        else:
            self.speed_limit_count = 0
            
            
        if (self.speed_limit_count == 21): # Greater than 20 times
            # pass
            led_cmd = Bool()
            led_cmd.data = True
            self.led.publish(led_cmd)


        
        self.speed = max(0, min(self.speed, self.get_parameter('max_throttle').value)) # Limit the throttle
        

def main(args=None):
    rclpy.init(args=args)
    config_file = sys.argv[1] if len(sys.argv) > 1 else None
    node = LaneGuidanceNode(config_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
