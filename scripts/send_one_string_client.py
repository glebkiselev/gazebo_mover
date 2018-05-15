#!/usr/bin/env python

import rospy
from crumb_planner.srv import *

def send_one_string_client(string_message):
    rospy.wait_for_service('send_one_string')
    try:
        send_one_string = rospy.ServiceProxy('send_one_string', SendOneString)
        resp1 = send_one_string(string_message)
        return resp1.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":

    string_message = 'move;8,2;above'

    send_one_string_client(string_message)