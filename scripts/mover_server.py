#!/usr/bin/env python

from crumb_planner.srv import *
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import std_srvs
from std_srvs.srv import Empty
import time
from math import pow, atan2, sqrt, acos, atan, radians
from tf import transformations as trans
import numpy as np

prev_directions = []


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

        # odom subscriber
        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        self.__angle = float()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
     received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 6)
        self.pose.y = round(self.pose.y, 6)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def move(self, pose_x, pose_y, direction):
        """Moves the turtle to the goal."""
        print('poses are {0},{1}'.format(pose_x, pose_y))

        goal_pose = Point()

        goal_pose.x = pose_x
        goal_pose.y = pose_y
        goal_pose.z = 0

        resp = self.rotate_to_goal(goal_pose, direction)

        while not resp:
            time.sleep(1)

        # get previous angle to goal moving
        angle = resp[1]
        clockwise = resp[2]
        if clockwise == True:
            clockwise = False
        else:
            clockwise = True

        vel_msg = Twist()

        vel_msg.linear.x = 1
        # Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        # TODO test that on 1 line or move diagonal/katiti
        while not rospy.is_shutdown():

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0
            evcl = self.euclidean_distance(goal_pose)
            # Loop to move the turtle in an specified distance
            while (current_distance < evcl):
                # print('i am in {0},{1}'.format(self.pose.x, self.pose.y))
                # Publish the velocity
                self.velocity_publisher.publish(vel_msg)
                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                current_distance = vel_msg.linear.x * (t1 - t0)
            # After the loop, stops the robot
            vel_msg.linear.x = 0
            # Force the robot to stop
            self.velocity_publisher.publish(vel_msg)
            print('robot stoped')
            # Rotate to previous direction
            PI = 3.1415926535897
            speed = 7.0
            angular_speed = speed * 2 * PI / 360
            resp2 = self.rudder(angle*2, angular_speed, clockwise)
            if resp2 == 'yes':
                return 'yes'

        # If we press control + C, the node will stop.
        rospy.spin()

    def rotate_to_goal(self, goal_pose, direction):
        print('rotating to goal!')
        # hypotenuse = self.euclidean_distance(goal_pose)

        if goal_pose.x > self.pose.x and goal_pose.y > self.pose.y:
            if goal_pose.x > goal_pose.y:
                new_dir = 'above'
                clockwise = False
            else:
                new_dir = 'left'
                clockwise = True
        elif goal_pose.x < self.pose.x and goal_pose.y > self.pose.y:
            if abs(goal_pose.x) > abs(goal_pose.y):
                new_dir = 'below'
                clockwise = True
            else:
                new_dir = 'left'
                clockwise = False
        elif goal_pose.x > self.pose.x and goal_pose.y < self.pose.y:
            if abs(goal_pose.x) > abs(goal_pose.y):
                new_dir = 'above'
                clockwise = True
            else:
                new_dir = 'right'
                clockwise = False
        else:
            if abs(goal_pose.x) > abs(goal_pose.y):
                new_dir = 'below'
                clockwise = False
            else:
                new_dir = 'right'
                clockwise = True

        if abs(goal_pose.x) - abs(self.pose.x) > abs(goal_pose.y) - abs(self.pose.y):
            nearest = abs(abs(self.pose.x) - abs(goal_pose.x))
            other = abs(abs(self.pose.y) - abs(goal_pose.y))
        else:
            nearest = abs(abs(self.pose.y) - abs(goal_pose.y))
            other = abs(abs(self.pose.x) - abs(goal_pose.x))

        print('len of nearest is {0}'.format(nearest))
        print('len of other is {0}'.format(other))

        angle = atan(other / nearest) / 2

        if new_dir != direction:
            print('new dir is {0}'.format(new_dir))
            resp = self.rotate(new_dir, direction)
        else:
            resp = 'yes'

        # PI = 3.1415926535897
        speed = 2.0

        if resp:
            angular_speed = radians(speed)

            resp2 = self.rudder(angle, angular_speed, clockwise)

            if resp2:
                return resp2, angle, clockwise

    def get_angle(self, direct, prev_direct):
        circle = {}
        circle['above'] = 0
        circle["below"] = 180
        circle["left"] = -90
        circle["right"] = 90
        circle["above-left"] = -45
        circle["above-right"] = 45
        circle["below-left"] = -135
        circle["below-right"] = 135

        prev_angle = circle[prev_direct]
        dir_angle = circle[direct]
        if prev_angle < 0 and dir_angle > 0:
            ch_ang = abs(prev_angle) + dir_angle
        elif prev_angle < 0 and dir_angle < 0:
            ch_ang = dir_angle - prev_angle
        elif prev_angle > 0 and dir_angle < 0:
            ch_ang = prev_angle - dir_angle
        else:
            ch_ang = dir_angle - prev_angle
        return ch_ang

    def __odom_handler(self, msg):
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        a = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        self.__angle = a

    def rudder(self, relative_angle, angular_speed=0.1, clockwise=True):

        # print('rotating to {0}, clockwise {1}, speed {2}'.format(relative_angle, clockwise, angular_speed))

        vel_msg = Twist()
        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        # t0 = self.time
        current_angle = 0

        print('cur start ang is {0}'.format(current_angle))
        print('odom angle start %s' % self.__angle)

        while (current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        print('wish angle is {0}'.format(relative_angle))
        print('cur finish angle is {0}, time is {1}'.format(current_angle, rospy.Time.now().to_sec()))
        print('odom angle finish %s' % self.__angle)

        # Forcing our robot to stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return 'yes'

    def rotate(self, direct, prev_direct):
        "rotate to new direction"
        print('rotating to {0}'.format(direct))
        PI = 3.1415926535897

        # degrees/sec
        speed = 5.0
        # direction
        print('new dir: {0}, prev dir: {1}'.format(direct, prev_direct))
        angle = self.get_angle(direct, prev_direct)

        if angle < 0:
            clockwise = False
        else:
            clockwise = True

        angle = abs(angle)

        # Converting from angles to radians
        angular_speed = speed * 2 * PI / 360
        relative_angle = angle * 2 * PI / 360

        return self.rudder(relative_angle, angular_speed, clockwise)

        # rospy.spin()

    def pickup(self, x):
        print('pick-up call')
        return 'yes'


def handle_action(req):
    # print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    if req.action:
        # print("req on the server")
        print(req.action)

    act_form = [a.strip() for a in req.action.split(";")]

    resp = 'No'

    try:
        x = TurtleBot()
        global prev_directions
        if not prev_directions:
            prev_direct = 'above'
            prev_directions.append(prev_direct)
        else:
            prev_direct = prev_directions[-1]
        if act_form[0] == 'move':
            string_point = act_form[1]
            gp_x = int(string_point.split(",")[0])
            gp_y = int(string_point.split(",")[1])
            resp = x.move(gp_x, gp_y, prev_direct)
        elif act_form[0] == 'rotate':

            resp = x.rotate(act_form[2], prev_direct)
        elif act_form[0] == 'pick-up':
            resp = x.pickup(act_form)
    except rospy.ROSInterruptException:
        pass

    return SendOneStringResponse(resp)


def move_server():
    rospy.init_node('send_one_string_server')

    empty_call = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    empty_call()

    s = rospy.Service('send_one_string', SendOneString, handle_action)
    print("Ready to move.")
    rospy.spin()


if __name__ == "__main__":
    move_server()
