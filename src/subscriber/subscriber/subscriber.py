#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

    def twist_callback(self, msg):
        throttle = msg.linear.x  
        steer = msg.angular.z     
        dumper = msg.linear.y    
        bucket = msg.linear.z    

        command = f"T:{throttle},D:{dumper},B:{bucket},S:{steer}\n"
        self.ser.write(command.encode('utf-8'))

        self.get_logger().info(
            f"Sent → T:{throttle:.2f}, D:{dumper:.2f}, B:{bucket:.2f}, S:{steer:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

