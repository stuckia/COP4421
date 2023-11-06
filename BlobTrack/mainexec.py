import rclpy
import random
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unitysim_msgs.msg import BoundingBox3d

class laserdrive(Node):

  def __init__(self):
    super().__init__('laserdrive')
    self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
    self.sub1 = self.create_subscription(BoundingBox3d, 'healthfinder', self.health_callback, 10)
    self.sub2 = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
    self.laserdata = None
    self.minangle = 0
    self.maxangle = 0
    self.deltaangle = 0
    self.maxrange = 0
    self.lastblobsize = 0
    self.blobpicked = 0
    self.bloby = 0
    self.blobsize = 0
    self.timer = self.create_timer(0.1, self.move)

  def health_callback(self, msg):
    self.bloby = msg.center.position.y
    self.blobsize = msg.size.y
    
    if(self.lastblobsize >= 0.9 and self.blobsize <= 0.2):
      self.lastblobsize = self.blobsize
      self.blobpicked = 1
    elif(self.blobsize == 0):
      self.lastblobsize = 0
      self.blobpicked = 0
    else:
      self.blobpicked = 0

  def laser_callback(self, msg):
    self.laserdata = msg.ranges
    self.minangle = msg.angle_min
    self.maxangle = msg.angle_max
    self.deltaangle = msg.angle_increment
    self.maxrange = msg.range_max
    for i in range(0, len(self.laserdata)):
      if math.isinf(self.laserdata[i]):
        self.laserdata[i] = self.maxrange

  def getmindist(self, max_ang, min_ang):
    min_index = self.angletoindex(min_ang * math.pi / 180)
    max_index = self.angletoindex(max_ang * math.pi / 180)
    return min(self.laserdata[min_index:max_index]) / (max_index - min_index)

  def getmaxdist(self, max_ang, min_ang):
    min_index = self.angletoindex(min_ang * math.pi / 180)
    max_index = self.angletoindex(max_ang * math.pi / 180)
    return max(self.laserdata[min_index:max_index]) / (max_index - min_index)

  def getavgdist(self, max_ang, min_ang):
    min_index = self.angletoindex(min_ang * math.pi / 180)
    max_index = self.angletoindex(max_ang * math.pi / 180)
    avg = 0
    for i in range(min_index, max_index, 1):
      avg += self.laserdata[i]
    avg /= (max_index - min_index)
    avg /= self.maxrange
    return avg

  def getrightsum(self):
    rightsum = 0
    count = 0
    for i in range(self.angletoindex(-90 * math.pi / 180), self.angletoindex(-10 * math.pi / 180), 1):
      rightsum += self.laserdata[i]
      count += 1
    rightsum /= count
    rightsum /= self.maxrange
    return rightsum

  def getleftsum(self):
    leftsum = 0
    count = 0
    for i in range(self.angletoindex(10 * math.pi / 180), self.angletoindex(90 * math.pi / 180), 1):
      leftsum += self.laserdata[i]
      count += 1
    leftsum /= count
    leftsum /= self.maxrange
    return leftsum

  def blobtrack(self):
    msg = Twist()
    msg.linear.x = 0.8
    if(self.bloby > 0):
      msg.angular.z = -0.25 * self.blobsize * math.pi
      self.lastblobsize = self.blobsize
      self.pub.publish(msg)
      msg.angular.z = 0.0
      self.pub.publish(msg)
    elif(self.bloby < 0):
      msg.angular.z = 0.25 * self.blobsize * math.pi
      self.lastblobsize = self.blobsize
      self.pub.publish(msg)
      msg.angular.z = 0.0
      self.pub.publish(msg)
    else:
      self.safewander()
      
    self.bloby = 0

  def safewander(self):
    #fwd movement
    fwdsum = 1000
    count = 0
    for i in range(self.angletoindex(-10 * math.pi / 180),self.angletoindex(10 * math.pi / 180), 1):
      if (fwdsum > self.laserdata[i]):
        #min range in front arc
        fwdsum = self.laserdata[i]
        #avg arc -> fwdsum/=count
        #normalize via max dist
        fwdsum /= self.maxrange

        rightsum = self.getrightsum()
        leftsum = self.getleftsum()

        msg = Twist()
        msg.linear.x = fwdsum *2
        msg.angular.z = (leftsum - rightsum) * math.pi * 2
        self.pub.publish(msg)

  def turn_right(self):
    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = 0.0
    
    while(self.getmindist(90,-90) != self.getmindist(-30,-90)):
      msg.linear.x = 0.7
      msg.angular.z = -0.8
      self.pub.publish(msg)
      
  def turn_left(self):
    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = 0.0
    
    while(self.getmindist(90,-90) != self.getmindist(90,30)):
      msg.linear.x = 0.7
      msg.angular.z = 0.8
      self.pub.publish(msg)

  def wall_right(self):
    desired_dist = 1
    current_dist = self.getmindist(-30,-90)
    diff = desired_dist - current_dist
    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = 0.0

    if (current_dist < desired_dist):
      msg.linear.x = 1.0
      msg.angular.z = 0.5

    self.pub.publish(msg)

  def wall_left(self):
    desired_dist = 1
    current_dist = self.getmindist(90,30)
    diff = desired_dist - current_dist
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.0

    if (current_dist < desired_dist):
      msg.linear.x = 1.0
      msg.angular.z = 0.5

    self.pub.publish(msg)

  def move(self):
    if self.laserdata is not None:
      if (self.getmindist(90, -90) == self.getmindist(30, -30)):
        #minimum is in front, pick either R or L wall follow
        if (self.getrightsum() > self.getleftsum()):
          self.turn_right()
          self.wall_right()
        elif (self.getleftsum() > self.getrightsum()):
          self.turn_left()
          self.wall_left()
        else:
          #fwd movement
          fwdsum = 1000
          count = 0
          for i in range(self.angletoindex(-10 * math.pi / 180),self.angletoindex(10 * math.pi / 180), 1):
            if (fwdsum > self.laserdata[i]):
              #min range in front arc
              fwdsum = self.laserdata[i]
          #avg arc -> fwdsum/=count
          #normalize via max dist
          fwdsum /= self.maxrange

          rightsum = self.getrightsum()
          leftsum = self.getleftsum()

          msg = Twist()
          msg.linear.x = fwdsum
          msg.angular.z = (leftsum - rightsum) * math.pi * 2
          self.pub.publish(msg)
      else:
        if(self.blobpicked == 1):
          self.safewander()
        elif(self.blobsize > 0.1):
          self.blobtrack()
        else:
          self.safewander()

  def angletoindex(self, angle):
    #in rad
    return math.floor((angle - self.minangle) / self.deltaangle)

  def indextoangle(self, index):
    return (index * self.deltaangle) + self.minangle


def main(args=None):
  rclpy.init(args=args)
  mover = laserdrive()
  rclpy.spin(mover)
  dp.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
