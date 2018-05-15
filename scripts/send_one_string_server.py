#!/usr/bin/env python

from crumb_planner.srv import *
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg  import Point
from math import pow,atan2,sqrt


class TurtleBot:

 def __init__(self):
     # Creates a node with name 'turtlebot_controller' and make sure it is a

     # Publisher which will publish to the topic '/turtle1/cmd_vel'.
     self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity',
                                               Twist, queue_size=10)

     # A subscriber to the topic '/simulation/pose'. self.update_pose is called
     # when a message of type Point is received.
     self.pose_subscriber = rospy.Subscriber('/simulation/pose',
                                             Point, self.update_pose)

     self.pose = Point()
     self.rate = rospy.Rate(10)

 def update_pose(self, data):
     """Callback function which is called when a new message of type Pose is
     received by the subscriber."""
     self.pose = data
     self.pose.x = round(self.pose.x, 4)
     self.pose.y = round(self.pose.y, 4)

 def euclidean_distance(self, goal_pose):
     """Euclidean distance between current pose and the goal."""
     return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                 pow((goal_pose.y - self.pose.y), 2))


 def move2goal(self, pose_x, pose_y):
     """Moves the turtle to the goal."""
     print(pose_x)
     print(pose_y)

     goal_pose = Point()

     goal_pose.x = pose_x
     goal_pose.y = pose_y
     goal_pose.z = 0

     # Please, insert a number slightly greater than 0 (e.g. 0.01).
     #distance_tolerance = 0.01

     vel_msg = Twist()

     vel_msg.linear.x = 1
     # Since we are moving just in x-axis
     vel_msg.linear.y = 0
     vel_msg.linear.z = 0
     vel_msg.angular.x = 0
     vel_msg.angular.y = 0
     vel_msg.angular.z = 0
     #TODO test that on 1 line or move diagonal/katiti
     while not rospy.is_shutdown():

         # Setting the current time for distance calculus
         t0 = float(rospy.Time.now().to_sec())
         current_distance = 0

         # Loop to move the turtle in an specified distance
         while (current_distance < self.euclidean_distance(goal_pose)):
             # Publish the velocity
             self.velocity_publisher.publish(vel_msg)
             # Takes actual time to velocity calculus
             t1 = float(rospy.Time.now().to_sec())
             # Calculates distancePoseStamped
             current_distance = 1 * (t1 - t0)
         # After the loop, stops the robot
         vel_msg.linear.x = 0
         # Force the robot to stop
         self.velocity_publisher.publish(vel_msg)
         # # IZMENIL TUT'
         # rospy.spin()

     # If we press control + C, the node will stop.
     rospy.spin()

def handle_action(req):
    #print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    if req.action:
        print("req on the server")
    string_point = req.action.split(";")[1]
    gp_x = int(string_point.strip().split(",")[0])
    gp_y = int(string_point.strip().split(",")[1])
    try:
        x = TurtleBot()
        x.move2goal(gp_x, gp_y)
    except rospy.ROSInterruptException:
        pass

    return SendOneStringResponse('yes')

def move_server():
    rospy.init_node('send_one_string_server')
    s = rospy.Service('send_one_string', SendOneString, handle_action)
    print("Ready to move.")
    rospy.spin()


if __name__ == "__main__":
    move_server()