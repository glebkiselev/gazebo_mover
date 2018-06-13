#!/usr/bin/env python

import rospy
from crumb_planner.srv import *
import argparse
from planner.spatial_planner import search_plan
from planner.ros_connector.processer import Processer
import time

def send_one_string_client(string_message):
    rospy.wait_for_service('send_one_string')
    try:
        send_one_string = rospy.ServiceProxy('send_one_string', SendOneString)
        resp1 = send_one_string(string_message)
        if resp1.reply:
            print("YES REPLY")
        time.sleep(1)
        return resp1.reply
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":

    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(dest='problem')
    argparser.add_argument('-s', '--saveload', action='store_true')

    args = argparser.parse_args()

    solution = search_plan(args.problem, args.saveload)

    pr = Processer(10, 10, 20)
    plan_to_file = []
    print("GOT THE SOLUTION. START SENDING...")



    delim = ';'
    i= 0
    maxtime = 600
    pr_time = 0

    for act in solution:
        new_coords = pr.to_gazebo(*act[1])
        plan_to_file.append(act[0] + delim + str(new_coords[0]) + ',' + str(new_coords[1]) + delim+ act[2])
    while i != len(plan_to_file):
        print(plan_to_file[i])
        reply = send_one_string_client(plan_to_file[i])
        while pr_time < maxtime:
            if not reply:
                time.sleep(10)
                pr_time+=10
            else:
                i+=1
                break



                #string_message = 'move;8,2;above'

    #send_one_string_client(string_message)