import logging

import itertools
from copy import deepcopy, copy

import planner.grounding.sign_task as st
from planner.grounding.semnet import Sign
import random
from planner.grounding.spatial_cognition.map_signs import state_prediction
from planner.grounding.spatial_cognition.map_signs import define_situation
from planner.grounding.spatial_cognition.map_signs import signs_markup
from planner.grounding.spatial_cognition.map_signs import define_map
from planner.grounding.spatial_cognition.map_signs import update_situation

MAX_ITERATION = 60

world_model = None
constraints = None


def map_search(task):
    global world_model
    global constraints
    world_model = task.signs
    check_pm = task.goal_situation.meanings[1]
    active_pm = task.start_situation.meanings[1]
    constraints = task.constraints
    map_pms = task.map_precisions
    check_map = task.goal_map.meanings[1]
    logging.debug('Start: {0}'.format(check_pm.longstr()))
    logging.debug('Finish: {0}'.format(active_pm.longstr()))
    plans = map_iteration(active_pm, check_pm, map_pms, check_map, [], 0, task.additions)
    if plans:
        solution = long_relations(plans)
        #solution.reverse()
    else:
        logging.info('No solution can be found!')
        return None
    return solution


def change_map(map_pms, cell_location):
    pms = map_pms.spread_down_activity('meaning', 4)
    pm_list = []
    contain_reg = None
    for location, cells in cell_location.items():
        if 'cell-4' in cells:
            contain_reg = location
    for iner in pms:
        iner_names = [s.sign.name for s in iner]
        if 'include' in iner_names:
            pm_list.append(iner[-1])
    for pm in pm_list:
        if pm.sign.name != 'cell-4':
            if pm.sign.name == contain_reg:
                return False
    return True


def _step_generating(active_pm, map_pms, script, agent, additions, iteration, param, prev_state):
    next_pm, cell_coords, parsed_map, cell_location, near_loc, region_map, direction = _state_prediction(active_pm, script, agent, additions,iteration, param)
    prev_state.append(cell_coords['cell-4'])
    additions[0][iteration+1] = cell_coords
    additions[1][iteration+1] = parsed_map
    if change_map(map_pms, cell_location):
        map_pms = define_map(st.MAP_PREFIX + str(st.SIT_COUNTER), region_map, cell_location, near_loc, additions[2], world_model)
        print('map has been changed!')
        #em = map_pms.spread_down_activity('meaning', 4)
    #TODO compare vs additions[3] - cell_map
    elif iteration > 0:
        if list(additions[3][iteration].values()) != list(additions[3][iteration-1].values()):
            map_pms = define_map(st.MAP_PREFIX + str(st.SIT_COUNTER), region_map, cell_location, near_loc, additions[2],
                                 world_model)
            print('block was moved!')

    return next_pm, map_pms, prev_state, direction


def map_iteration(active_pm, check_pm, map_pms, check_map, current_plan, iteration, additions, prev_state = [], exp_actions=[]):
    logging.debug('STEP {0}:'.format(iteration))
    logging.debug('\tSituation {0}'.format(active_pm.longstr()))
    if iteration >= MAX_ITERATION:
        logging.debug('\tMax iteration count')
        return None

    precedents = []
    plan_signs = []
    I_sign, I_obj, agents = get_agents()

    for name, sign in world_model.items():
        if name.startswith("action_"): plan_signs.append(sign)
        for index, cm in sign.meanings.items():
            if cm.includes('meaning', active_pm):
                precedents.extend(cm.sign.spread_up_activity_act('meaning', 1))
            elif not cm.sign.significances and active_pm.includes('meaning', cm):
                precedents.extend(cm.sign.spread_up_activity_act('meaning', 1))

    active_chains = active_pm.spread_down_activity('meaning', 4)
    active_signif = set()

    for chain in active_chains:
        pm = chain[-1]
        active_signif |= pm.sign.spread_up_activity_act('significance', 3)

    meanings = []
    for pm_signif in active_signif:
        chains = pm_signif.spread_down_activity('significance', 6)
        merged_chains = []
        for chain in chains:
            for achain in active_chains:
                if chain[-1].sign == achain[-1].sign and len(chain) > 2 and chain not in merged_chains:
                    merged_chains.append(chain)
                    break
        scripts = _generate_meanings(merged_chains, agents)
        meanings.extend(scripts)
    applicable_meanings = []
    agent = None
    if not precedents:
        for agent, cm in meanings:
            result, checked = _check_activity(cm, active_pm)
            if result:
                applicable_meanings.append((agent, checked))
    else:
        for cm in precedents + meanings:
            if isinstance(cm, tuple):
                agent = cm[0]
                cm = cm[1]
            result, checked = _check_activity(cm, active_pm)
            if result:
                applicable_meanings.append((agent, checked))

    # TODO: replace to metarule apply
    candidates = _meta_check_activity(applicable_meanings, active_pm, check_pm, [x for x, _, _, _, _ in current_plan], additions, iteration, prev_state, check_map)

    if not candidates:
        logging.debug('\tNot found applicable scripts ({0})'.format([x for _, x, _, _,_ in current_plan]))
        return None

    logging.debug('\tFound {0} variants'.format(len(candidates)))
    final_plans = []


    print("len of curent plan is: {0}. Len of candidates: {1}".format(len(current_plan), len(candidates)))

    for counter, name, script, ag_mask in candidates:
        #todo remember previous coords
        logging.debug('\tChoose {0}: {1} -> {2}'.format(counter, name, script))
        plan = copy(current_plan)
        #plan.append((active_pm, name, script, ag_mask))

        next_pm, map_pms, prev_state, direction = _step_generating(active_pm, map_pms, script, agent, additions, iteration, True, prev_state)
        ag_place = (prev_state[-1][2] - prev_state[-1][0]) // 2 + prev_state[-1][0], (prev_state[-1][3] - prev_state[-1][1]) // 2 + prev_state[-1][1]
        print(ag_place)
        plan.append((active_pm, name, script, ag_mask, (ag_place, direction)))

        if next_pm.includes('meaning', check_pm):
            # if next_pm.includes('meaning', check_pm):
            #     pass
            if map_pms.includes('meaning', check_map):
                final_plans.append(plan)
                plan_actions = [x.sign.name for _, _, x, _, _ in plan]
                print("len of final plan is: {0}".format(len(plan)))
                print(plan_actions)
        else:
            recursive_plans = map_iteration(next_pm, check_pm, map_pms, check_map, plan, iteration + 1, additions, prev_state, exp_actions)
            if recursive_plans:
                # maxLen = len(recursive_plans[0])
                final_plans.extend(recursive_plans)

    return final_plans


def _get_experience(agents):
    actions = []
    for agent in agents:
        for connector in agent.out_meanings:
            cm = connector.in_sign.meanings[connector.in_index]
            if max([len(event.coincidences) for event in itertools.chain(cm.cause, cm.effect)]) > 1:
                if cm.is_causal():
                    #for sign
                    for pm in actions:
                        if pm.resonate('meaning', cm):
                            break
                    else:
                        actions.append(cm)

    return actions


def long_relations(plans):
    logging.info("in long relations")
    # min(len) plans
    min = len(plans[0])
    for plan in plans:
        if len(plan) < min:
            min = len(plan)
    plans = [plan for plan in plans if len(plan) == min]

    busiest = []
    for index, plan in enumerate(plans):
        previous_agent = ""
        agents = {}
        counter = 0
        plan_agents = []
        #lens of following actions from 1 ag
        for action in plan:
            if action[3] not in agents:
                agents[action[3]] = 1
                previous_agent = action[3]
                counter = 1
                if not action[3] is None:
                    plan_agents.append(action[3].name)
                else:
                    plan_agents.append(str(action[3]))
            elif not previous_agent == action[3]:
                previous_agent = action[3]
                counter = 1
            elif previous_agent == action[3]:
                counter += 1
                if agents[action[3]] < counter:
                    agents[action[3]] = counter
        # max queue of acts
        longest = 0
        agent = ""
        for element in range(len(agents)):
            item = agents.popitem()
            if item[1] > longest:
                longest = item[1]
                agent = item[0]
        busiest.append((index, agent, longest, plan_agents))
    cheap = []
    alternative = []
    cheapest = []
    longest = 0
    min_agents = 100
    # max queue
    for plan in busiest:
        if plan[2] > longest:
            longest = plan[2]
    # min(len(ag))
    for plan in busiest:
        if plan[2] == longest:
            if len(plan[3]) < min_agents:
                min_agents = len(plan[3])
    # if I in
    for plan in busiest:
        if plan[3][0]:
            if plan[2] == longest and len(plan[3]) == min_agents and "I" in plan[3]:
                plans_copy = copy(plans)
                cheap.append(plans_copy.pop(plan[0]))
            elif plan[2] == longest and len(plan[3]) == min_agents and not "I" in plan[3]:
                plans_copy = copy(plans)
                alternative.append(plans_copy.pop(plan[0]))
        else:
            plans_copy = copy(plans)
            cheap.append(plans_copy.pop(plan[0]))
    if len(cheap) >= 1:
        cheapest.extend(random.choice(cheap))
    elif len(cheap) == 0 and len(alternative):
        logging.info("There are no plans in which I figure")
        cheapest.extend(random.choice(alternative))

    return cheapest


def get_agents():
    agent_back = set()
    I_sign = world_model['I']
    agent_back.add(I_sign)
    I_obj = [con.in_sign for con in I_sign.out_significances if con.out_sign.name == "I"][0]
    They_sign = world_model["They"]
    agents = They_sign.spread_up_activity_obj("significance", 1)
    for cm in agents:
        agent_back.add(cm.sign)
    return I_sign, I_obj, agent_back


def _check_experience(candidates, exp_actions):
    actions = []
    for candidate in candidates:
        if candidate[3]:
            if candidate[2] in exp_actions:
                actions.append(candidate)
    if actions:
        return actions
    else:
        return candidates


def mix_pairs(replace_map):
    new_chain = {}
    elements = []
    merged_chains = []
    used_roles = []
    replace_map = list(replace_map.items())

    def get_role(obj, roles):
        for role in roles:
            if obj in role[1]:
                return role

    for item in replace_map:
        elements.append(item[1])
    elements = list(itertools.product(*elements))
    clean_el = copy(elements)
    for element in clean_el:
        if not len(set(element)) == len(element):
            elements.remove(element)
    for element in elements:
        for obj in element:
            avalaible_roles = [x for x in replace_map if x not in used_roles]
            role = get_role(obj, avalaible_roles)
            if role:
                used_roles.append(role)
                new_chain[role[0]] = obj
        merged_chains.append(new_chain)
        new_chain = {}
        used_roles = []
    return merged_chains




def _generate_meanings(chains, agents):
    def get_role_index(chain):
        index = 0
        rev_chain = reversed(chain)
        for el in rev_chain:
            if len(el.cause) == 0:
                continue
            elif len(el.cause) == 1:
                index = chain.index(el)
            else:
                return index
        return None

    def get_big_role_index(chain):
        index = None
        for el in chain:
            if len(el.cause) == 1:
                index = chain.index(el)
                break
            else:
                continue
        if index:
            return index
        return None

    big_replace = {}

    replace_map = {}
    main_pm = None
    for chain in chains:
        role_index = get_role_index(chain)
        if role_index:
            if not chain[role_index].sign in replace_map:
                replace_map[chain[role_index].sign] = [chain[-1]]
            else:
                if not chain[-1] in replace_map[chain[role_index].sign]:
                    replace_map[chain[role_index].sign].append(chain[-1])
        role_index = get_big_role_index(chain)
        if role_index:
            if not chain[role_index].sign in big_replace:
                big_replace[chain[role_index].sign] = [chain[role_index + 1]]
            else:
                if not chain[role_index + 1] in big_replace[chain[role_index].sign]:
                    big_replace[chain[role_index].sign].append(chain[role_index + 1])
            main_pm = chain[0]

    connectors = [agent.out_meanings for agent in agents]

    main_pm_len = len(main_pm.cause) + len(main_pm.effect) + 2

    unrenewed = {}
    for agent_con in connectors:
        for con in agent_con:
            if con.in_sign == main_pm.sign:
                unrenewed.setdefault(con.out_sign, set()).add(con.in_sign.meanings[con.in_index])


    new_map = {}

    rkeys = {el for el in replace_map.keys()}


    pms = []
    for agent, lpm in unrenewed.items():
        # firstly full signed actions from experience
        for pm in lpm.copy():
            if len(pm.cause) + len(pm.effect) != main_pm_len:
                continue
            pm_signs = set()
            pm_mean = pm.spread_down_activity('meaning', 3)
            for pm_list in pm_mean:
                pm_signs |= set([c.sign for c in pm_list])
            role_signs = rkeys & pm_signs
            if not role_signs:
                lpm.remove(pm)
                if not pms:
                    pms.append((agent, pm))
                else:
                    for _, pmd in copy(pms):
                        if pmd.resonate('meaning', pm):
                            break
                    else:
                        pms.append((agent, pm))
        old_pms = []

        if len(pms) == 64: break

        for pm in lpm:
            if len(pm.cause) + len(pm.effect) != main_pm_len:
                continue
            pm_signs = set()
            pm_mean = pm.spread_down_activity('meaning', 3)
            for pm_list in pm_mean:
                pm_signs |= set([c.sign for c in pm_list])
            if pm_signs not in old_pms:
                old_pms.append(pm_signs)
            else:
                continue
            role_signs = rkeys & pm_signs
            for role_sign in role_signs:
                new_map[role_sign] = replace_map[role_sign]


            # search for changed in grounding
            # old_map = {}
            # changed_event = []
            # changed_signs = set()
            # for key, value in replace_map.items():
            #     if key not in new_map:
            #         old_map[key] = value
            # for key, value in old_map.items():
            #     value_signs = {cm.sign for cm in value}
            #     for event in itertools.chain(pm.cause, pm.effect):
            #         event_signs = event.get_signs()
            #         changed_sign = event_signs & value_signs
            #         if changed_sign:
            #             changed_signs |= changed_sign
            #             changed_event.append((event_signs - changed_signs))
            #
            # predicates = set()
            # for changed_sign in changed_signs:
            #     predicates |= {cm.sign for cm in changed_sign.spread_up_activity_obj('meaning', 1)}
            # # search for pred name where changed
            # predicates_signs = set()
            # for element in changed_event:
            #     predicates_signs |= predicates & element
            #
            # # search for object signs to change
            # if agent.name != "I":
            #     agent_name = agent.name
            # else:
            #     agent_name = list(agent.spread_up_activity_obj('significance', 1))[0].sign.name
            # if constraints and agent_name in constraints:
            #     agent_predicates = [pred for pred in constraints[agent_name] if world_model[pred.name] in predicates_signs]
            # else:
            #     agent_predicates = []
            # predicates_signatures = []
            # for pred in agent_predicates:
            #     predicates_signatures.append([signa[0] for signa in pred.signature])
            # #predicates_objects = {world_model[signa[0]] for signa in predicates_signatures}
            # changed = []
            # for sign in changed_signs:
            #     changed = [signa for signa in predicates_signatures if sign.name in signa]
            # predicates_objects = set()
            # for sign in changed_signs:
            #     for signa in changed:
            #         for element in signa:
            #             if element != sign.name:
            #                 predicates_objects.add(world_model[element])
            #
            # if changed_event:
            #     changed_event = reduce(lambda x, y: x|y, changed_event)
            #
            # for key, item in new_map.copy().items():
            #     if key in changed_event:
            #         new_dict = {}
            #         if predicates_objects:
            #             new_dict[key] = [cm for cm in new_map[key] if cm.sign in predicates_objects]
            #             if not new_dict[key]:
            #                 new_dict[key] = new_map[key]
            #             new_map.update(new_dict)
                # new_dict = {}
                # new_dict[key]=[cm for cm in item if cm.sign != world_model["I"]]
                # new_map.update(new_dict)



            #TODO take more complex closely()
            #
            # mchains = pm.spread_down_activity('meaning', 3)

            for chain in pm_mean:
                if chain[-1].sign in big_replace and not chain[-1].sign in new_map :
                    for cm in big_replace.get(chain[-1].sign):
                        if world_model['cell?x'] in cm.get_signs() and world_model['cell?y'] in cm.get_signs():
                            new_map[chain[-1].sign] = [cm]

            ma_combinations = mix_pairs(new_map)

            for ma_combination in ma_combinations:
                cm = pm.copy('meaning', 'meaning')

                for role_sign, obj_pm in ma_combination.items():
                    obj_cm = obj_pm.copy('significance', 'meaning')
                    # role_sign = [sign for sign in cm.get_signs() if sign == role_sign][0]
                    cm.replace('meaning', role_sign, obj_cm)
                #TODO replace

                for matr in cm.spread_down_activity('meaning', 6):
                    if matr[-1].sign.name == 'cell?y' or matr[-1].sign.name == 'cell?x':
                        celly = world_model['cell?y']
                        cellx = world_model['cell?x']
                        cell_y_change = ma_combination[celly]
                        cm.replace('meaning', celly, cell_y_change)
                        cell_x_change = world_model['cell-4'].add_meaning()
                        cm.replace('meaning', cellx, cell_x_change)
                        break
                # if world_model['cell?y'] in cms:
                #     celly = ma_combination[world_model['cell?y']]
                #     cm.replace('meaning', world_model['cell?y'], celly)
                # if world_model['cell?x'] in cms:
                #     if world_model['cell?x'] in ma_combination:
                #         # cellx = ma_combination[world_model['cell?x']]
                #         cellx = world_model['cell-4'].add_meaning()
                #     else:
                #         cellx = world_model['cell-4'].add_meaning()
                #     cm.replace('meaning', world_model['cell?x'], cellx)
                #cms = cm.spread_down_activity('meaning', 6)


                if not pms:
                    pms.append((agent, cm))
                else:
                    for _, pmd in copy(pms):
                        if pmd.resonate('meaning', cm):
                            break
                    else:
                        pms.append((agent, cm))
            if len(old_pms) == 64:
                break

    return pms

def watch_matrix(event, base):
    matrices = []
    for connector in event.coincidences:
        matrices.append(getattr(connector.out_sign, base+'s')[connector.out_index])

    return matrices

def _check_activity(pm, active_pm):
    result = True
    for event in pm.cause:
        #matrices = watch_matrix(event, 'meaning')
        for fevent in active_pm.cause:
            #fmatrices = watch_matrix(fevent, 'meaning')
            if event.resonate('meaning', fevent):
                break
        else:
            result = False
            break

    if not result:
        expanded = pm.expand('meaning')
        if not len(expanded.effect) == 0:
            return _check_activity(expanded, active_pm)
        else:
            return False, pm
    return result, pm


def _time_shift_backward(active_pm, script):
    next_pm = Sign(st.SIT_PREFIX + str(st.SIT_COUNTER))
    world_model[next_pm.name] = next_pm
    pm = next_pm.add_meaning()
    st.SIT_COUNTER += 1
    copied = {}
    for event in active_pm.cause:
        for es in script.effect:
            if event.resonate('meaning', es):
                break
        else:
            pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
    for event in script.cause:
        pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
    return pm

def _time_shift_forward(active_pm, script):
    next_pm = Sign(st.SIT_PREFIX + str(st.SIT_COUNTER))
    world_model[next_pm.name] = next_pm
    pm = next_pm.add_meaning()
    st.SIT_COUNTER += 1
    copied = {}
    for event in active_pm.cause:
        for es in script.cause:
            if event.resonate('meaning', es):
                break
        else:
            pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
    for event in script.effect:
        pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
    return pm

def _state_prediction(active_pm, script, agent, additions, iteration, flag = False):
    direction = None
    cell = None
    events = []
    fast_estimation = _time_shift_forward(active_pm, script)
    orientation = fast_estimation.get_iner(world_model['orientation'], 'meaning')[0]
    employment = fast_estimation.get_iner(world_model['employment'], 'meaning')[0]
    holding = None
    h_sign = world_model['holding']
    if h_sign in fast_estimation.get_signs():
        holding = fast_estimation.get_iner(h_sign, 'meaning')[0]
    for sign in orientation.get_signs():
        if sign!=agent:
            direction = sign
    for sign in employment.get_signs():
        if sign!=agent:
            cell = sign
    for ev in fast_estimation.cause:
        if len(ev.coincidences) == 1:
            for con in ev.coincidences:
                if con.out_sign.name == "I":
                    events.append(ev)
        elif not holding:
            if "I" in [s.name for s in ev.get_signs()]:
                events.append(ev)
    agent_state = state_prediction(agent, direction, holding)
    cell_coords = deepcopy(additions[0][iteration][cell.name])
    new_x_y = deepcopy(additions[1][iteration])
    new_x_y['objects']['agent']['x'] = (cell_coords[2] - cell_coords[0]) // 2 + cell_coords[0]
    new_x_y['objects']['agent']['y'] = (cell_coords[3] - cell_coords[1]) // 2 + cell_coords[1]
    #for pick-up script
    if holding:
        block_name = [sign.name for sign in holding.get_signs() if 'block' in sign.name][0]
        if block_name in new_x_y['objects'].keys():
            new_x_y['objects'][block_name]['y'] = new_x_y['objects']['agent']['y']
            new_x_y['objects'][block_name]['x'] = new_x_y['objects']['agent']['x']
        else:
            block = {}
            block[block_name] = {'x':new_x_y['objects']['agent']['x'], 'y': new_x_y['objects']['agent']['y'], 'r': 5}
            new_x_y['objects'].update(block)
            #print()
    #for put-down script
    if script.sign.name == 'put-down':
        block_name = [sign.name for sign in script.get_iner(world_model['holding'], 'meaning')[0].get_signs() if 'block' in sign.name][0]
        table_name = [sign.name for sign in script.get_iner(world_model['ontable'], 'meaning')[0].get_signs() if sign.name!=block_name][0]
        new_x_y['objects'][block_name]['y'] = new_x_y['objects'][table_name]['y']
        new_x_y['objects'][block_name]['x'] = new_x_y['objects'][table_name]['x']
    region_map, cell_map, cell_location, near_loc, cell_coords_new = signs_markup(new_x_y, 'agent', cell_coords)

    estimation = define_situation(fast_estimation.sign.name+'sp', cell_map, events, agent_state, world_model)
    estimation = update_situation(estimation, cell_map, world_model)

    if flag:
        region = None
        for reg, cellz in cell_location.items():
            if 'cell-4' in cellz:
                region = reg
        additions[3][iteration] = cell_map
        print("act: {0}, cell: {1}, dir: {2}, reg: {3}".format(script.sign.name, cell_coords_new['cell-4'], direction.name, region))
        return estimation, cell_coords_new, new_x_y, cell_location, near_loc, region_map, direction.name

    return estimation, cell_coords_new, new_x_y, cell_location, near_loc, region_map


def get_stright(estimation, dir_sign):
    es = estimation.spread_down_activity('meaning', 4)
    grouped = {}
    for key, group in itertools.groupby(es, lambda x: x[1]):
        for pred in group:
            grouped.setdefault(key, []).append(pred[-1])
    stright_cell = None
    used_key = None
    for key, item in grouped.items():
        it_signs = {am.sign for am in item}
        if dir_sign in it_signs:
            stright_cell = [sign for sign in it_signs if sign.name != 'cell-4' and sign != dir_sign and sign.name != "I"]
            if stright_cell:
                stright_cell = stright_cell[0]
                used_key = key
                break
    for key, item in grouped.items():
        if key != used_key:
            it_signs_names = {am.sign.name for am in item}
            if stright_cell.name in it_signs_names:
                if 'nothing' in it_signs_names:
                    return None
                else:
                    items = [it for it in item if it.sign != stright_cell and it.sign.name != 'cell-4']
                    if items: return items






def _meta_check_activity(scripts, active_pm, check_pm, prev_pms, additions, iteration, prev_state, check_map):
    heuristic = []
    for agent, script in scripts:
        estimation, cell_coords_new, new_x_y,cell_location, near_loc, region_map = _state_prediction(active_pm, script, agent, additions, iteration)

        if not new_x_y['objects']['agent']['x'] in range(0, new_x_y['map_size'][0]) or \
               not new_x_y['objects']['agent']['y'] in range(0, new_x_y['map_size'][1]):
            break

        for prev in prev_pms:
            if estimation.resonate('meaning', prev, False, False):
                if cell_coords_new['cell-4'] in prev_state:
                    break
        else:
            counter = 0
            # for event in [event for event in estimation.cause if "I" not in event.get_signs_names()]:
            #     for ce in [event for event in check_pm.cause if "I" not in event.get_signs_names()]:
            #         if event.resonate('meaning', ce):
            #             counter += 1
            #             break
            cont_region = None
            future_region = None
            for reg, cellz in cell_location.items():
                if 'cell-4' in cellz:
                    cont_region = reg
            agent_sign = world_model['agent']
            for iner in check_map.get_iner(world_model['contain'], 'meaning'):
                iner_signs = iner.get_signs()
                if agent_sign in iner_signs:
                    for sign in iner_signs:
                        if sign != agent_sign and 'region' in sign.name:
                            future_region = sign
                            break
            if future_region.name != cont_region:
                ag_orient = estimation.get_iner(world_model['orientation'], 'meaning')[0]
                iner_signs = ag_orient.get_signs()
                dir_sign = None
                for sign in iner_signs:
                    if sign != world_model["I"]:
                        dir_sign = sign
                normal_dir = additions[2][cont_region][future_region.name][1]
                stright = get_stright(estimation, dir_sign)
                if dir_sign.name == normal_dir:
                    if stright:
                        counter=0
                    else:
                        counter+=2
                #for move action
                elif cell_coords_new['cell-4'] != additions[0][iteration]['cell-4']:
                    counter+=1
                else:
                    # check clously to goal region regions
                    closely_goal = [reg for reg, ratio in additions[2][future_region.name].items() if
                                    ratio[0] == 'closely']
                    closely_dirs = set()
                    if cont_region not in closely_goal:
                        for region in closely_goal:
                            closely_dirs.add(additions[2][cont_region][region][1])
                        if dir_sign.name in closely_dirs:
                            if stright:
                                counter = 0
                            else:
                                counter += 1
                    else:
                        closely_dir = additions[2][cont_region][future_region.name][1]
                        if dir_sign.name == closely_dir:
                            counter += 2

            else:
                est_events = [event for event in estimation.cause if "I" not in event.get_signs_names()]
                ce_events = [event for event in check_pm.cause if "I" not in event.get_signs_names()]
                for event in est_events:
                    for ce in ce_events:
                        if event.resonate('meaning', ce):
                            counter += 1
                            break

            heuristic.append((counter, script.sign.name, script, agent))
    if heuristic:
        best_heuristics = max(heuristic, key=lambda x: x[0])
        return list(filter(lambda x: x[0] == best_heuristics[0], heuristic))
    else:
        return None
