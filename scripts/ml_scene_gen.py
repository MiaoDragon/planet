"""
Generate task planning instances for ML.
generate:
- XML files to define the start poses
- PDDL files defining the start semantics

Here we simply use the 5-object scene, and randomly pick for each object:
- color
- which table it's on (table 1 or table 2)
- pose (within certain boundary of the table)

We make sure they are not in collision with each other by enforcing radius threshold

table1 boundary:
[-0.35,0.35]+2.2, [-0.75,0.75], 0.767
table2 boundary:
[-0.35,0.35]-2.2, [-0.75,0.75], 0.767

each object shape: -0.02~0.02 x 0.2
Hence we use radius threshold of at least 0.05 to ensure they are not colliding
"""
import numpy as np
from .data_gen_util import *

def scene_gen(num_problems, sp, path):
    # path: the directory to store the output
    # will store at path/{id}/

    table1_low = np.array([-0.35+2.2, -0.75, 0.767])
    table1_high = np.array([0.35+2.2, 0.75, 0.767])
    table2_low = np.array([-0.35-2.2, -0.75, 0.767])
    table2_high = np.array([0.35-2.2, 0.75, 0.767])
    padding = 0.2
    col_threshold = 0.1
    #num_problems = 1
    num_objs = 4

    problem_list = []
    while len(problem_list) != num_problems:
        obs_list = {}
        color_choices = ['blue', 'green']
        table_choices = ['table1', 'table2']
        for color in color_choices:
            obs_list[color] = []
        obs_list_total = []
        obs_names_list = []
        table_low = {'table1': table1_low, 'table2': table2_low}
        table_high = {'table1': table1_high, 'table2': table2_high}
        while len(obs_names_list) != num_objs:
            # randomly pick color
            obs_color = np.random.randint(low=0, high=len(color_choices))
            obs_color = color_choices[obs_color]
            # randomly pick table
            obs_table = np.random.randint(low=0, high=len(table_choices))
            obs_table = table_choices[obs_table]
            # generate obs_name, which is going to be used in xml pose
            obs_name = obs_color+str(len(obs_list[obs_color]))
            # generate obs pose
            low = table_low[obs_table]
            high = table_high[obs_table]
            obs_loc = np.random.uniform(low[:2]+padding, high[:2]-padding)
            obs_loc = np.append(obs_loc, [low[2]], axis=0)
            # make sure the pose is not colliding with others by checking the distance
            invalid = False
            for past_obs in obs_list_total:
                past_obs_loc = past_obs['pose'][:,3]
                if np.linalg.norm(obs_loc - past_obs_loc) < col_threshold:
                    invalid = True
                    break
            if invalid:
                continue
            
            # collision-free
            obs_pose = np.zeros([3,4])
            obs_pose[0,0] = 1
            obs_pose[1,1] = 1
            obs_pose[2,2] = 1
            obs_pose[:,3] = obs_loc

            obs_dict = {}
            obs_dict = {'color': obs_color, 'table': obs_table, 'name': obs_name, 'pose': obs_pose}
            obs_list[obs_color].append(obs_dict)
            obs_list_total.append(obs_dict)
            obs_names_list.append(obs_name)

        # construct problem dict
        problem = {}
        problem['name'] = 'prob'+str(len(problem_list)+sp)
        problem['obs'] = obs_list
        problem['obs_list'] = obs_names_list

        problem_list.append(problem)

        save_path = path + "%d/" % (len(problem_list)-1+sp)
        import os
        os.makedirs(save_path, exist_ok=True)
        # construct xml and pddl files
        
        # pddl
        pddl_str = dict_to_pddl(problem)
        pddl_file = open(save_path+"clutter_%s.pddl" % (problem['name']), 'w')
        pddl_file.write(pddl_str)
        pddl_file.close()

        # xml
        #xml_tree = dict_to_xml(problem)
        #xml_tree.write('clutter_%s.xml' % (problem['name']))
        xml_str = dict_to_xml(problem, path)
        xml_file = open(save_path+'clutter_%s.xml' % (problem['name']), 'w')
        xml_file.write(xml_str)
        xml_file.close()

        # toml
        # dict_to_toml(save_path, problem['name'], path)
