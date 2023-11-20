import rclpy
import random
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import String

class laserdrive(Node):
  def __init__(self):
    super().__init__('laserdrive')
    self.lasersub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
    self.laserdata = None
    self.minangle = 0
    self.maxangle = 0
    self.deltaangle = 0
    self.maxrange = 0
    self.lastnoninfvalue = 16.0

  def laser_callback(self, msg):
    self.laserdata = msg.ranges
    self.minangle = msg.angle_min
    self.maxangle = msg.angle_max
    self.deltaangle = msg.angle_increment
    self.maxrange = msg.range_max
    for i in range(0, len(self.laserdata)):
      if math.isinf(self.laserdata[i]):
        if(16.0 - self.lastnoninfvalue > 8.0):
          self.laserdata[i] = 16.0
        else:
          self.laserdata[i] = 0.0
      self.lastnoninfvalue = self.laserdata[i]
    self.printvals()

  def angletoindex(self, angle):
    #in rad
    return math.floor((angle - self.minangle) / self.deltaangle)

  def indextoangle(self, index):
    return (index * self.deltaangle) + self.minangle
    
  def printvals(self):
    self.get_logger().info("Range at 0: " + str(self.laserdata[self.angletoindex(0)]))
    self.get_logger().info("Range at 90: " + str(self.laserdata[self.angletoindex(1.571)]))
    self.get_logger().info("Range at -90: " + str(self.laserdata[self.angletoindex(-1.571)]))
    self.get_logger().info("Range at " + str(self.maxangle) + ": " + str(self.laserdata[self.angletoindex(self.maxangle)]))


def main(args=None):
  rclpy.init(args=args)
  mover = laserdrive()
  rclpy.spin(mover)
  dp.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
