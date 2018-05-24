import json
from ros_connector.processer import Processer

def parse_world(file):
    book = []
    elements = {}
    with open(file, 'r+') as f:
        book.extend(f.readlines())
    for elem in book:
        if '<model name=' in elem:
            name = elem.split('\'')[1]
            if not 'ground' in name:
                if not name in elements:
                    pose = None
                    mass = None
                    for elem2 in book[book.index(elem)+1:len(book)-1]:
                        if 'pose' in elem2:
                            pose = elem2[12:-7].split(' ')
                        if 'mass' in elem2:
                            mass = float(elem2[16:-8])
                        if pose and mass:
                            elements[name] = (pose[0], pose[1], mass)
                            break
    return elements

def start_creater(map_name, elements, ex):

    pr = Processer(10, 10, 20)

    table = None
    block = None
    maxim = 0
    for code, el in elements.items():
        if el[2] > maxim:
            table = code, *pr.to_signs(float(el[0]),float(el[1]))
            maxim = el[2]
    for code, el in elements.items():
        if code != table[0]:
            block = code, *pr.to_signs(float(el[0]),float(el[1]))

    def identical():
        map = {}
        map['borders'] = {'border-1': {'y1': -20, 'x1': -20, 'y2': -20, 'x2': 220}}
        bd2 = {'border-2': {'y1': -20, 'x1': -20, 'y2': 200, 'x2': -20}}
        bd3 = {'border-3': {'y1': 220, 'x1': -20, 'y2': 220, 'x2': 220}}
        bd4 = {'border-4': {'y1': -20, 'x1': 220, 'y2': 220, 'x2': 220}}
        map['borders'].update(bd2)
        map['borders'].update(bd3)
        map['borders'].update(bd4)
        map['locations'] = {'location1': {'y1': 0, 'x1': 0, 'y2': 200, 'x2': 200}}
        map['map_size'] = [200, 200]
        return map


    if map_name == 'start_map':
        start_map = {}
        start_map['agent-actuator'] = {'agent':['I'], 'handempty': {'cause':[], 'effect':[]}}
        start_map['agent-orientation'] = 'above'
        start_map['objects'] = {'agent': {'x': 100, 'y': 100, 'r':10}}
        bl_a = {'block-a':{'x':block[1], 'r':5, 'y':block[2]}}
        tb_1 = {'table-1':{'x':table[1], 'r': 20, 'y': table[2]}}
        start_map['objects'].update(bl_a)
        start_map['objects'].update(tb_1)
        old = identical()
        start_map.update(old)
        with open('planner/spatial-benchmarks/task6/start_sit.json', 'w') as data_file1:
            json.dump(start_map, data_file1, sort_keys=True, indent=4)
    if map_name == 'finish_map':
        finish_map = {}
        finish_map['agent-actuator'] = {'holding':{'cause':['I', 'block-a'], 'effect':[]}}
        finish_map['agent-orientation'] = 'left'
        finish_map['objects'] = {'agent': {'x': table[1]+5, 'y': table[2]+5, 'r':10}}
        tb_1 = {'table-1':{'x':table[1], 'r': 20, 'y': table[2]}}
        finish_map['objects'].update(tb_1)
        old = identical()
        finish_map.update(old)
        with open('planner/spatial-benchmarks/task6/finish_sit.json', 'w') as data_file2:
            json.dump(finish_map, data_file2, sort_keys=True, indent=4)


if __name__ == "__main__":
    file = 'planner/spatial-benchmarks/task6/pick_place.world'
    elem = parse_world(file)
    with open('planner/spatial-benchmarks/task5/finish_sit.json') as data_file1:
        finish_map = json.load(data_file1)
    start_creater('finish_map', elem, finish_map)
    print()