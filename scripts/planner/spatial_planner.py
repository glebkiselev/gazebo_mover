#!/usr/bin/env python
import argparse
import json
import os
from search.mapsearch import map_search
from grounding.spatial_cognition.map_signs import ground
from ros_connector.processer import Processer
import pickle

def search_plan(problem, saveload):
    # dir = os.getcwd() + '/src/crumb_planner/scripts/planner/'
    # domain = dir+problem + 'spatial_domain.json'
    # start_sit = dir+problem + 'start_sit.json'
    # finish_sit = dir+problem + 'finish_sit.json'
    # #print(os.listdir('/src/crumb_planner/scripts/planner/'))
    # print(dir)

    domain = problem + 'spatial_domain.json'
    start_sit = problem + 'start_sit.json'
    finish_sit = problem + 'finish_sit.json'



    with open(start_sit) as data_file1:
        start_map = json.load(data_file1)
    with open(finish_sit) as data_file2:
        finish_map = json.load(data_file2)
    with open(domain) as data_file3:
        signs_structure = json.load(data_file3)
    agent = 'agent'
    task = ground(start_map, finish_map, signs_structure, agent)
    solution = map_search(task)

    plan = [(x.sign.name, y[0], y[1]) for _, _, x, _, y in solution]

    return plan


if __name__ == "__main__":

    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(dest='problem')
    argparser.add_argument('-s', '--saveload', action='store_true')

    args = argparser.parse_args()

    solution = search_plan(args.problem, args.saveload)
    #print(solution)

    pr = Processer(10, 10, 20)
    plan_to_file = []

    start_dir = 'above'
    delim = ';'

    for act in solution:
        new_coords = pr.to_gazebo(*act[1])
        plan_to_file.append(act[0]+delim+str(new_coords[0])+','+str(new_coords[1])+act[2])
    with open('data.pickle', 'wb') as f:
        pickle.dump(plan_to_file, f)
#
#

