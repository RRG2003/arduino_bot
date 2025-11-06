#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import Quaterniontoeuler, Eulertoquaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class AngleConversion(Node):
    def __init__(self):
        super().__init__("angle_conversion_service")

        self.euler_to_quaternion_ = self.create_service(Eulertoquaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_= self.create_service(Quaterniontoeuler,"quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Service is ready.")

    def eulerToQuaternionCallback(self, req, res):
        self.get_logger().info("Received Euler angles: roll=%f, pitch=%f, yaw=%f" % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Converted to Quaternion: x=%f, y=%f, z=%f, w=%f" % (res.x, res.y, res.z, res.w))
        return res
    
    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info("Received Quaternion: x=%f, y=%f, z=%f, w=%f" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info("Converted to Euler angles: roll=%f, pitch=%f, yaw=%f" % (res.roll, res.pitch, res.yaw))
        return res
    
def main():
    rclpy.init()
    angle_conversion_service = AngleConversion()
    rclpy.spin(angle_conversion_service)
    angle_conversion_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
