import rclpy
import random

from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class movement(Node):
	def __init__(self):
		super().__init__('canyonrun_exec')
		self.pub = self.create_publisher(Twist,"cmd_vel",10)
		self.leftsub = self.create_subscription(Int32,"left",self.left,10)
		self.rightsub = self.create_subscription(Int32,"right",self.right,10)
		# publisher for R sensor
		# publisher for L sensor
		self.isleft = 0
		self.isright = 0
		self.timer = self.create_timer(.1, self.move)
		self.time = 0
		self.state = 0
		self.statechanger = 0;
		self.leftsub
		self.rightsub
	
	def left(self,msg):
		if(msg.data == 1):
			self.state = 2
			self.isleft = 1
		else:
			self.isleft = 0
	
	def right(self,msg):
		if(msg.data == 1):
			self.state = 1
			self.isright = 1
		else:
			self.isright = 0
	
	def move(self):
		msg = Twist()
		
		if(self.isright == 1):
			if(self.isleft == 1):
				self.state = 3
		
		if(self.state == 0):
			# move fwd
			msg.linear.x = 1.0
		elif(self.state == 1):
			# turn right and go back
			msg.linear.x = -1.0
			msg.angular.z = 1.0
			self.state = 0
		elif(self.state == 2):
			# turn left and go back
			msg.linear.x = -1.0
			msg.angular.z = -1.0
			self.state = 0
		elif(self.state == 3):
			msg.angular.z = -1.0
		
		self.pub.publish(msg)

# if hit wall on L and then R x number of times in x seconds -> declare reaching end wall, turn 180

def main(args=None):
	rclpy.init(args=args)
	movebot = movement()
	rclpy.spin(movebot)
	movebot.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
