import logging
import cv2
import numpy as np
import itertools

from grounding.semnet import Sign

def locater(map, location_name, centers):
    dislocations = {}
    place_map = {}
    itera = 0
    start_x = 0
    start_y = 0
    if isinstance(map, np.ndarray):
        blocksize= int(map.shape[1] //3), int(map.shape[0] /3)
    elif isinstance(map, list):
        blocksize = int((map[2] - map[0])/3), int((map[3] - map[1])/3)
        start_x = map[0]
        start_y = map[1]
    for i in range(3):
        for j in range(3):
            fy = j * blocksize[1] + blocksize[1] + start_y
            fx = i*blocksize[0] +blocksize[0] + start_x
            dislocations[location_name+str(itera)] = [i * blocksize[0]+start_x, j*blocksize[1]+start_y, fx, fy]
            itera+=1
    for lkey, lvalue in dislocations.items():
        for ckey, cvalue in centers.items():
            for value in cvalue:
                if lvalue[0] <= value[1][0] <= lvalue[2] and lvalue[1] <= value[1][1] <= lvalue[3]:
                    place_map.setdefault(lkey, set()).add(value[0])
                else:
                    place_map.setdefault(lkey, set()).add(0)

    return dislocations, place_map


def placer(image, block_size):
    """
    find squares of free space
    :param image:
    :param block_size:
    :return:
    """
    place = []
    for i in range(image.shape[0] // block_size):
        for j in range(image.shape[1] // block_size):
            local_place = []
            colors_sum = 0
            for ii in range(i * block_size, i * block_size + block_size):
                for jj in range(j * block_size, j * block_size + block_size):
                    colors_sum += image[ii][jj][0]
                    local_place.append((jj, ii))
            colors_sum /= block_size * block_size
            if colors_sum == 255:
                place.append(local_place)
    return place

def belonging(cell, reg):
    Rx = (cell[2] - cell[0]) //2
    Ry = (cell[3] - cell[1]) //2
    a = reg[0] <= cell[0] and reg[1] <= cell[1]
    b = reg[2] >= cell[2] and reg[3] >= cell[3]
    if a and b:
        return True
    elif cell[0] <= reg[0] and cell[1] >= reg[1] and b:
        if reg[0] - cell[0] <= Rx:
            return True
    elif cell[0] >= reg[0] and cell[1] <= reg[1] and b:
        if reg[1] - cell[1] <= Ry:
            return True
    elif a and cell[2] >= reg[2] and cell[3] <= reg[3]:
        if cell[2] - reg[2] <= Rx:
            return True
    elif a and cell[2] <= reg[2] and cell[3] >= reg[3]:
        if cell[3] - reg[3] <= Ry:
            return True
    elif cell[0] <= reg[0] and cell[1] <= reg[1] and b:
        if reg[1] - cell[1] <= Ry and reg[0] - cell[0] <= Rx:
            return True
    elif cell[2] >= reg[2] and cell[3] >= reg[3] and a:
        if cell[2] - reg[2] <= Rx and cell[3] - reg[3] <= Ry:
            return True
    elif reg[0] < cell[0] and reg[1] > cell[1] and reg[2] < cell[2] and reg[3] > cell[3]:
        if cell[2] - reg[2] <= Rx and reg[1] -cell[1] <= Ry:
            return True
    elif reg[0] > cell[0] and reg[1] < cell[1] and reg[2] > cell[2] and reg[3] < cell[3]:
        if reg[0] - cell[0] <= Rx and cell[3] - reg[3]<= Ry:
            return True
    return False

def cell_creater(size, obj_loc, region_location):
    cell_loc = {}
    ysize = size[3] - size[1]
    xsize = size[2] - size[0]
    new_region = [size[0]-xsize, size[1]-ysize, size[2]+xsize, size[3]+ysize]
    cell_location, cell_map = locater(new_region, 'cell', obj_loc)
    for cell, cdisl in cell_location.items():
        for region, rdisl in region_location.items():
            if belonging(cdisl, rdisl):
                cell_loc.setdefault(region, []).append(cell)
                break
        else:
            cell_loc.setdefault('border', []).append(cell)

    return cell_loc, cell_map


def size_founder(reg_loc, obj_loc, ag):
    others = set()
    target = None
    others.add(ag)
    others.add(0)
    dislocations, place_map = locater(reg_loc, 'proto-cell', obj_loc)
    for cell, filling in place_map.items():
        if ag in filling:
            others = filling - others
            target = cell
            break
    if others:
        size = size_founder(dislocations[target], obj_loc, ag)
    else:
        size = dislocations[target]
    #TODO while objects and agents have radius >= cell/2
    # TODO 2 obj too clear 1 to another
    return size


def map_recogn(map_path, types, agent):
    #place objects on map
    """
    :param map_path: image of map
    :param types: possible objects, which includes tables, blocks, agents
    :return ndarray and dict: in which keys - object designation in value tuple (object name, centre coords)
    """
    obj_loc = {}
    block_size = 20
    elements = 5
    # map to ndarray
    image = cv2.imread(map_path)

    # dict form
    place = placer(image, block_size)
    pl = list(np.random.choice(len(place)-1, elements, replace=False))

    subList = [pl[i::len(types)] for i in range(len(types))]
    subList.reverse()

    for ind, type in enumerate(types):
        pl_num = subList[ind]
        for num in pl_num:
            centre = round(place[num][0][0]+(block_size*0.5)), round(place[num][0][1]+(block_size*0.5))
            obj_loc.setdefault(type, []).append((type+str(len(obj_loc[type])), centre))

    # add agent
    ag_centre = [i for i in place if not i in subList][0]
    centre = round(ag_centre[0][0] + (block_size * 0.5)), round(ag_centre[0][1] + (block_size * 0.5))
    obj_loc.setdefault(agent, []).append((agent, centre))
    # division into regions
    region_location, region_map = locater(image, 'region', obj_loc)

    # division into cells
    # cell size finding
    size = 0
    for key, value in region_map.items():
        if agent in value:
            size = size_founder(region_location[key], obj_loc, agent)
            break
    # TODO cell_map test
    cell_location, cell_map = cell_creater(size, obj_loc, region_location)

    return region_map, cell_map, cell_location

def ground(map_file, object_types, agent):
    """
    :param
    :return: Task
    """
    # map recognition
    region_map, cell_map, cell_location = map_recogn(map_file, object_types, agent)

    obj_signifs = {}
    obj_means = {}
    signs = {}
    #I, They signs
    I_sign = Sign("I")
    They_sign = Sign("They")
    obj_signifs[I_sign] = I_sign.add_significance()
    signs[I_sign.name] = I_sign
    obj_signifs[They_sign] = They_sign.add_significance()
    signs[They_sign.name] = They_sign
    # create sign map and matrix
    Map_sign = Sign("Map")
    obj_signifs[Map_sign] = Map_sign.add_significance()
    signs[Map_sign.name] = Map_sign
    # create regions signs and matrixes
    Region_sign = Sign("Region")
    region_signif = Region_sign.add_significance()
    obj_signifs[Region_sign] = region_signif
    signs[Region_sign.name] = Region_sign
    regions = []
    for role in ('?X', '?Y', '?Z'):
        Region_signs = Sign('Region'+role)
        regions.append(Region_signs)
        role_signif = Region_signs.add_significance()
        obj_signifs[Region_signs] = role_signif
        signs[Region_signs.name] = Region_signs
        connector = role_signif.add_feature(region_signif, zero_out=True)
        Region_sign.add_out_significance(connector)

    for region in region_map:
        Regions_sign = Sign(region)
        obj_signifs[Regions_sign] = Regions_sign.add_significance()
        signs[Regions_sign.name] = Regions_sign
        Region_signif = Region_sign.add_significance()
        connector = Region_signif.add_feature(obj_signifs[Regions_sign], zero_out=True)
        Regions_sign.add_out_significance(connector)


    # create cell signs and matrixes
    Cell_sign = Sign("Cell")
    obj_signifs[Cell_sign] = Cell_sign.add_significance()
    signs[Cell_sign.name] = Cell_sign

    Cellx_sign = Sign("Cell?X")
    obj_signifs[Cellx_sign] = Cellx_sign.add_significance()
    signs[Cellx_sign.name] = Cellx_sign

    Celly_sign = Sign("Cell?Y")
    obj_signifs[Celly_sign] = Celly_sign.add_significance()
    signs[Celly_sign.name] = Celly_sign

    Celly_signif = Celly_sign.add_significance()
    con = Celly_signif.add_feature(obj_signifs[Cell_sign], zero_out=True)
    Cell_sign.add_out_significance(con)

    for cell, value in cell_map.items():
        Cells_sign = Sign(cell)
        obj_signifs[Cells_sign] = Cells_sign.add_significance()
        signs[Cells_sign.name] = Cells_sign
        Cell_signif = Cell_sign.add_significance()
        connector = Cell_signif.add_feature(obj_signifs[Cells_sign], zero_out=True)
        Cells_sign.add_out_significance(connector)
        if agent in value:
            Cellx_signif = Cellx_sign.add_significance()
            con = Cellx_signif.add_feature(obj_signifs[Cells_sign], zero_out=True)
            Cells_sign.add_out_significance(con)



    # create objects signs
    Object_sign = Sign('Object')
    obj_signifs[Object_sign] = Object_sign.add_significance()
    signs[Object_sign.name] = Object_sign
    obj_signs = []
    Block_sign = Sign('Block')
    obj_signs.append(Block_sign)
    Obstacle_sign = Sign('Obstacle')
    obj_signs.append(Obstacle_sign)
    Border_sign = Sign('Border')
    obj_signs.append(Border_sign)
    Nothing_sign = Sign('Nothing')
    obj_signs.append(Nothing_sign)
    Agent_sign = Sign('Agent')
    obj_signs.append(Agent_sign)
    Table_sign = Sign('Table')
    obj_signs.append(Table_sign)
    for s in obj_signs:
        obj_signifs[s] = s.add_significance()
        signs[s.name] = s
        Object_sign_signif = Object_sign.add_significance()
        connector = Object_sign_signif.add_feature(obj_signifs[s], zero_out=True)
        s.add_out_significance(connector)

    dirs = []
    Direction_sign = Sign('Direction')
    dirs.append(Direction_sign)
    obj_signifs[Direction_sign] = Direction_sign.add_significance()
    signs[Direction_sign.name] = Direction_sign

    Direction_start = Sign('Dir-start')
    dirs.append(Direction_start)
    Dir_signif = Direction_start.add_significance()
    obj_signifs[Direction_start] = Dir_signif
    signs[Direction_start.name] = Direction_start
    connector = Dir_signif.add_feature(obj_signifs[Direction_sign], zero_out=True)
    Direction_sign.add_out_significance(connector)

    Direction_finish = Sign('Dir-finish')
    dirs.append(Direction_finish)
    Dir_signif = Direction_finish.add_significance()
    obj_signifs[Direction_finish] = Dir_signif
    signs[Direction_finish.name] = Direction_finish
    connector = Dir_signif.add_feature(obj_signifs[Direction_sign], zero_out=True)
    Direction_sign.add_out_significance(connector)

    directions = ['Above', 'Below', 'Left', 'Right', 'Above-left', 'Above-right', 'Below-left', 'Below-right']

    for dir in directions:
        dir_sign = Sign(dir)
        dir_signif = dir_sign.add_significance()
        obj_signifs[dir_sign] = dir_signif
        signs[dir_sign.name] = dir_sign
        Direction_signif = Direction_sign.add_significance()
        connector = Direction_signif.add_feature(dir_signif, zero_out=True)
        dir_sign.add_out_significance(connector)

    distance = []
    # locate objects: include
    Include_sign = Sign('Include')
    distance.append(Include_sign)
    signs[Include_sign.name] = Include_sign
    obj_signifs[Include_sign] = Include_sign.add_significance()
    for obj in obj_signs:
        Include_signif = Include_sign.add_significance()
        connector = Include_signif.add_feature(obj_signifs[Region_sign], effect=False, zero_out=True)
        Region_sign.add_out_significance(connector)
        connector = Include_signif.add_feature(obj_signifs[obj], effect=True, zero_out=True)
        obj.add_out_significance(connector)
    #TODO test random cell generation place
    for region, cells in cell_location.items():
        region_sign = signs[region]
        for cell in cells:
            Include_signif = Include_sign.add_significance()
            connector = Include_signif.add_feature(obj_signifs[region_sign], effect=False, zero_out=True)
            region_sign.add_out_significance(connector)
            cell_sign = signs[cell]
            connector = Include_signif.add_feature(obj_signifs[cell_sign], effect=True, zero_out=True)
            cell_sign.add_out_significance(connector)
    #include cellX cellY
    Include_X_signif = Include_sign.add_significance()
    connector = Include_X_signif.add_feature(obj_signifs[Region_sign], effect=False, zero_out=True)
    Region_sign.add_out_significance(connector)
    connector = Include_X_signif.add_feature(obj_signifs[Cellx_sign], effect=True, zero_out=True)
    Cellx_sign.add_out_significance(connector)
    Include_Y_signif = Include_sign.add_significance()
    connector = Include_Y_signif.add_feature(obj_signifs[Region_sign], effect=False, zero_out=True)
    Region_sign.add_out_significance(connector)
    connector = Include_Y_signif.add_feature(obj_signifs[Celly_sign], effect=True, zero_out=True)
    Celly_sign.add_out_significance(connector)
    # locate objects: contain
    ag_cell = []
    Contain_sign = Sign('Contain')
    contain_signif = Contain_sign.add_significance()
    obj_signifs[Contain_sign] = contain_signif
    for cell, objs in cell_map.items():
        cell_sign = signs[cell]
        if len(objs) > 1 and 0 in objs:
            for obj in objs:
                if obj !=0:
                    contain_signif = Contain_sign.add_significance()
                    connector = contain_signif.add_feature(obj_signifs[cell_sign], effect=False, zero_out=True)
                    cell_sign.add_out_significance(connector)
                    if obj in signs:
                        obj_sign = signs[obj]
                    else:
                        obj_sign = Sign(obj)
                        if 'agent' in obj:
                            ag_cell.append(cell)
                        signs[obj_sign.name] = obj_sign
                        obj_signifs[obj_sign] = obj_sign.add_significance()
                        obj_role = obj.title()[:-1]
                        role_names = [s.name for s in obj_signs]
                        if obj_role in role_names:
                            obj_role_sign = signs[obj_role]
                            orss = obj_role_sign.add_significance()
                            con = orss.add_feature(obj_signifs[obj_sign], zero_out=True)
                            obj_sign.add_out_significance(con)
                    connector = contain_signif.add_feature(obj_signifs[obj_sign], effect=True, zero_out=True)
                    obj_sign.add_out_significance(connector)

        else:
            contain_signif = Contain_sign.add_significance()
            connector = contain_signif.add_feature(obj_signifs[cell_sign], effect=False, zero_out=True)
            cell_sign.add_out_significance(connector)
            connector = contain_signif.add_feature(obj_signifs[Nothing_sign], effect=True, zero_out=True)
            Nothing_sign.add_out_significance(connector)

    # closely
    Closely_sign = Sign('Closely')
    distance.append(Closely_sign)
    signs[Closely_sign.name] = Closely_sign
    Closely_signif = Closely_sign.add_significance()
    obj_signifs[Closely_sign] = Closely_signif
    # locate cells: closely
    Closely_cells_signif = Closely_sign.add_significance()
    connector = Closely_cells_signif.add_feature(obj_signifs[Cellx_sign])
    Cellx_sign.add_out_significance(connector)
    connector = Closely_cells_signif.add_feature(obj_signifs[Celly_sign])
    Celly_sign.add_out_significance(connector)
    # locate regions: closely
    combinations = list(itertools.combinations(regions, 2))
    closely_regions = []
    for combination in combinations:
        Closely_signif = Closely_sign.add_significance()
        closely_regions.append(Closely_signif)
        connector = Closely_signif.add_feature(obj_signifs[combination[0]])
        combination[0].add_out_significance(connector)
        connector = Closely_signif.add_feature(obj_signifs[combination[1]])
        combination[1].add_out_significance(connector)
    # locate cell and regions: closely
    closely_regcells = []
    for reg in regions:
        Closely_signif = Closely_sign.add_significance()
        closely_regcells.append(Closely_signif)
        connector = Closely_signif.add_feature(obj_signifs[Cellx_sign])
        Cellx_sign.add_out_significance(connector)
        connector = Closely_signif.add_feature(obj_signifs[reg])
        reg.add_out_significance(connector)
    for reg in regions:
        Closely_signif = Closely_sign.add_significance()
        closely_regcells.append(Closely_signif)
        connector = Closely_signif.add_feature(obj_signifs[Celly_sign])
        Celly_sign.add_out_significance(connector)
        connector = Closely_signif.add_feature(obj_signifs[reg])
        reg.add_out_significance(connector)
    # nearly
    Nearly_sign = Sign('Nearly')
    distance.append(Nearly_sign)
    signs[Nearly_sign.name] = Nearly_sign
    Nearly_signif = Nearly_sign.add_significance()
    obj_signifs[Nearly_sign] = Nearly_signif
    # locate cell and regions: nearly
    cell_y_reg = [cm for cm in closely_regcells if Celly_sign in cm.get_signs()]
    for cm in cell_y_reg:
        Nearly_signif = Nearly_sign.add_significance()
        con = Nearly_signif.add_feature(Closely_cells_signif)
        Closely_sign.add_out_significance(con)
        conn = Nearly_signif.add_feature(cm)
        Closely_sign.add_out_significance(conn)

    # locate regions: nearly
    Nearly_signif_reg = Nearly_sign.add_significance()
    Region_X_sign = signs['Region?X']
    Region_Y_sign = signs['Region?Y']
    Region_Z_sign = signs['Region?Z']
    closely_XZ = [cm for cm in closely_regions if Region_X_sign in cm.get_signs() and Region_Z_sign in cm.get_signs()][0]
    closely_ZY = [cm for cm in closely_regions if Region_Z_sign in cm.get_signs() and Region_Y_sign in cm.get_signs()][0]
    con = Nearly_signif_reg.add_feature(closely_XZ)
    Closely_sign.add_out_significance(con)
    conn = Nearly_signif_reg.add_feature(closely_ZY)
    Closely_sign.add_out_significance(conn)

    # faraway
    Faraway_sign = Sign('Faraway')
    distance.append(Faraway_sign)
    signs[Faraway_sign.name] = Faraway_sign
    obj_signifs[Faraway_sign] = Faraway_sign.add_significance()
    Faraway_signif = Faraway_sign.add_significance()
    con = Faraway_signif.add_feature(Nearly_signif_reg)
    Nearly_sign.add_out_significance(con)
    conn = Faraway_signif.add_feature(Include_X_signif)
    Nearly_sign.add_out_significance(conn)

    # distance
    Distance_sign = Sign('Distance')
    signs[Distance_sign.name] = Distance_sign
    Distance_signif = Distance_sign.add_significance()
    obj_signifs[Distance_sign] = Distance_signif
    for dist in distance:
        Distance_signif = Distance_sign.add_significance()
        connector = Distance_signif.add_feature(obj_signifs[dist])
        dist.add_out_significance(connector)
    # location
    Location_sign = Sign('Location')
    signs[Location_sign.name] = Location_sign
    obj_signifs[Location_sign] = Location_sign.add_significance()
    for dir in dirs:
        Location_signif = Location_sign.add_significance()
        conn = Location_signif.add_feature(obj_signifs[Distance_sign], effect=False)
        Distance_sign.add_out_significance(conn)
        con = Location_signif.add_feature(obj_signifs[dir], effect=True)
        dir.add_out_significance(con)



if __name__ == "__main__":
    object_types = ["table", "block"]
    map_file = 'test.jpg'
    agent = 'agent0'
    ground(map_file, object_types, agent)