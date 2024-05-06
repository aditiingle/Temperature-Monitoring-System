#!/usr/bin/env python3

import rclpy  
from rclpy.node import Node

from example_interfaces.msg import String


class PythonNode(Node):


   def __init__(self):    
       super().__init__("py_node")

       self.robot_name_ = "PyPub"
       self.publisher_ = self.create_publisher(String, "py_topic", 10)
       self.subscriber_ = self.create_subscription(
           String, "cpp_topic", self.callback_news, 10)
       self.timer_ = self.create_timer(0.5, self.publish_news)
       self.get_logger().info("Python Node has been started.")


   def publish_news(self):
       msg = String()
       msg.data = "Hello from Python Publisher, " + str(self.robot_name_) + "."
       self.publisher_.publish(msg)
      
   def callback_news(self, msg):
       self.get_logger().info(msg.data)

def main(args=None):   
   rclpy.init(args=args)               
   node = PythonNode()  
   rclpy.spin(node)  
   rclpy.shutdown()   


if __name__ == "__main__":  
   main()
