#!/usr/bin/env python3

import rclpy  
from rclpy.node import Node
from std_msgs.msg import Float32

import random


class TemperatureReader(Node):


   def __init__(self):    
       super().__init__("reader_node")

       self.publisher_ = self.create_publisher(Float32, "temperature_topic", 10)
       self.subscriber_ = self.create_subscription(
           Float32, "alert_topic", self.callback_alert, 10)
       self.timer_ = self.create_timer(1.0, self.publish_temperature)
       self.get_logger().info("Temperature is being measured.")


   def publish_temperature(self):
       temperature = random.uniform(15.0, 40.0)
       msg = Float32()
       msg.data = temperature
       self.publisher_.publish(msg)
       self.get_logger().info(f"Published temperature: {temperature:.2f} degrees Celsius")
      
   def callback_alert(self, msg):
       if msg.data == 1.0:
        self.get_logger().warn("Alert: High temperature detected!")

def main(args=None):   
   rclpy.init(args=args)               
   node = TemperatureReader()  
   rclpy.spin(node)  
   rclpy.shutdown()   


if __name__ == "__main__":  
   main()
