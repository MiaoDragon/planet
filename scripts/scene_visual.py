"""
Read the scene described by xml file and use PyBullet to visualize
"""

import xml.etree.ElementTree as ET
import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
tree = ET.parse("../clutter_ml/clutter_prob0.xml")
robot = tree.find('robots').find('robot')
robot_pose = robot.find('basepose').text.split(' ')
robot_pose = [float(s) for s in robot_pose]
robot_pose = np.array(robot_pose).reshape((3,4))

robot_rot_mat = np.zeros((4,4))
robot_rot_mat[3,3] = 1
robot_rot_mat[:3,:3] = robot_pose[:,:3]
print(robot_rot_mat)
robot_ori = R.from_matrix(robot_pose[:,:3])
#robot_ori = tf.transformations.quaternion_from_matrix(robot_rot_mat)
robot_ori = robot_ori.as_quat()
print(robot_ori)
robot_pos = robot_pose[:,3]


import pybullet as p
bullet_client = p.connect(p.GUI)

# obtain robot position
import os
robot_model_filename = '../models/pr2.urdf'
robot_model_filename = os.path.abspath(robot_model_filename)
p.loadURDF(robot_model_filename, basePosition=robot_pos, baseOrientation=robot_ori)


# obtain objects
objs_node = tree.find('objects')
model_folder = '../clutter_ml/'
for obj_node in objs_node:
    # obj filename
    obj_filename = obj_node.find('geom').text
    obj_filename = model_folder + obj_filename
    obj_filename = os.path.abspath(obj_filename)
    obj_pose = obj_node.find('pose').text.split(' ')
    obj_pose = [float(s) for s in obj_pose]
    obj_pose = np.array(obj_pose).reshape((3,4))
    # orientation
    obj_pose_ori = R.from_matrix(obj_pose[:,:3])
    obj_pose_ori = obj_pose_ori.as_quat()
    obj_pose_pos = obj_pose[:,3]
    # load obj
    print('name: ', obj_filename)
    print('position: ')
    print(obj_pose_pos)
    print('orientation: ')
    print(obj_pose_ori)

    if 'blue' in obj_filename:
        rgba = [0,0,1,1]
    elif 'red' in obj_filename:
        rgba = [1,0,0,1]
    elif 'green' in obj_filename:
        rgba = [0,1,0,1]
    else:
        rgba = [1,1,1,1]
    idx = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_filename, rgbaColor=rgba)
    p.createMultiBody(basePosition=obj_pose_pos, baseOrientation=obj_pose_ori,
                      baseVisualShapeIndex=idx)
    # p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_filename, \
    #                rgbaColor=[1, 1, 1, 1], \
    #                specularColor=[0.4, .4, 0], \
    #                visualFramePosition=obj_pose_pos, visualFrameOrientation=obj_pose_ori)
    # p.createCollisionShape(shapeType=p.GEOM_MESH,
    #                     fileName=obj_filename,
    #                     collisionFramePosition=obj_pose_pos,
    #                     collisionFrameOrientation=obj_pose_ori)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while True:
    p.stepSimulation()
    time.sleep(0.1)
p.disconnect()
