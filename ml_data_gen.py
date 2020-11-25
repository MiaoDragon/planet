"""
Implement the pipeline to generate data
"""

import subprocess
from multiprocessing import Process, Queue

import argparse
from scripts.ml_scene_gen import scene_gen
from scripts.data_gen_util import toml_dict
import numpy as np
import os
def main(args):
    #scene_gen(args.NP, args.sp, args.path)
    
    # generate path
    path = args.path
    ps = []
    # 1 process -> 20 paths

    def single_process_gen(NP, sp):
        for i in range(NP):
            print('generating problem %d' % (i+sp))
            while True:
                # generate problems until success
                scene_gen(1, i+sp, args.path)

                prob_path = args.path + "%d/" % (i+sp)
                problem_name = "prob%d" % (i+sp)
                toml = toml_dict(prob_path, problem_name, path)
                # write the toml file
                f = open(prob_path + 'clutter_ml.toml', 'w')
                for name, val in toml.items():
                    f.write("%s = \"%s\"\n" % (name, val))
                f.close()
                # we give 5 trials, and if non succeed, then regenerate problem
                subprocess.run(args=["./planet_gen_traj", prob_path+"clutter_ml.toml", '-r', '3'],
                               )#stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

                # check if the solution path is stored, if yes, then success and generate next scene
                if os.path.exists(prob_path+'clutter_ml_cont_state.txt'):
                    break

    NP = 1 #20
    num_process = int(np.round(args.NP / NP))
    for i in range(num_process):
        # calculate sp
        sp = args.sp + NP * i
        p = Process(target=single_process_gen, args=(NP, sp))
        ps.append(p)
        p.start()

    try:
        for p in ps:
            p.join()
    except:
        print('terminating child processes...')
        for i in range(num_process):
            ps[i].terminate()
            ps[i].join()
        print('finished terminate.')            



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default='./')
    parser.add_argument('--NP', type=int, default=20)
    parser.add_argument('--sp', type=int, default=0)


    args = parser.parse_args()
    main(args)