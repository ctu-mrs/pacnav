#!/usr/bin/env python3
import argparse
import random
import yaml
import math

def getDist(
        p1,
        p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# #{ isColliding

def isColliding(
        pose,
        model_list,
        thresh_dist):

    count = 0
    for model_pose in model_list:
        if getDist(pose, model_pose) > thresh_dist:
            count += 1

    if count == len(model_list):
        return False
    else:
        return True

# #} end of isColliding

# get random safe position for model (try 100 times on collision)
# #{ getRandPose

def getRandPose(
        max_l,
        max_w,
        thresh_dist,
        model_list):

    counter = 0
    while counter < 100:
        cand_x = random.uniform((-max_l/2) - thresh_dist/2, (max_l/2) - thresh_dist/2)
        cand_y = random.uniform((-max_l/2) - thresh_dist/2, (max_l/2) - thresh_dist/2)
        pose = [cand_x, cand_y]

        if not isColliding(pose, model_list, thresh_dist):
            return pose
        
        counter += 1

    return None

# #} end of getRandPose

# #{ getAlphaLattice

def getAlphaLattice(
        max_l,
        max_w,
        thresh_dist):

    d = thresh_dist
    n_rows = int(max_w/(math.sqrt(3)*d/2)) + 1
    n_cols = int(max_l/d) + 1
    poses = []

    for i in range(0, n_rows):
        y = i*math.sqrt(3)*d/2 - max_w/2

        for j in range(0, n_cols):
            if i%2 == 0:
                x = j*d - max_l/2
            else:
                x = j*d + d/2 - max_l/2
            poses.append([x, y])

    return poses

# #}

# create world with randomly placed copies of model
# #{ createWorld

def createWorld(
        org_world,
        new_world,
        model,
        arena_w,
        arena_l,
        model_count,
        cur_config,
        uav_count,
        thresh_dist,
        mode="random"):

    uav_poses = []

    # #{ load uav poses from config file used for spawning

    if cur_config is not None:
        f = open(cur_config, 'r')

        uav_cfg = yaml.safe_load(f)

        # only load the poses of uavs used in the simulation
        for key in uav_cfg:
            pose = uav_cfg[key]
            uav_poses.append([pose['x'], pose['y']])

            if len(uav_poses) >= uav_count:
                break

        # print(uav_poses)

    # #} end of load uav poses and radius from config file used for spawning

    with open(org_world, 'r') as open_file:
        world_data = open_file.read()
    with open(model, 'r') as open_file:
        model_data = open_file.read()

    model_beg_idx = model_data.find('<model')
    model_end_idx = model_data.find('</sdf>')
    world_end_idx = world_data.find('</world>')

    lattice_poses = getAlphaLattice(arena_l, arena_w, thresh_dist)

    added_poses = []
    added_poses.extend(uav_poses)

    counter = 0
    tree_count = 0
    while (counter < model_count):

        # sample a random pose within the arena
        if mode == "random":
            tree_pose = getRandPose(arena_l, arena_w, thresh_dist, added_poses)
        else:
            tree_pose = lattice_poses[counter]
            
        counter += 1

        # add a model at the sampled pose
        if tree_pose != None:
            tree_count += 1
            added_poses.append(tree_pose)

            tmp_model = model_data[model_beg_idx:model_end_idx]
            tmp_model = tmp_model.replace('<model name="tree_simple">', '<model name="tree_simple'+str(counter)+'">')
            tmp_model = tmp_model.replace('<pose frame="">0 0 0 0 0 -0</pose>', '<pose frame=""> '+str(tree_pose[0])+' '+str(tree_pose[1])+' 0 0 0 0</pose>')
            world_data = world_data[:world_end_idx] + tmp_model + world_data[world_end_idx:]

    print('Trees spawned: {}'.format(tree_count))

    with open(new_world, 'w') as file :
        file.write(world_data)

# #} end of createWorld

if __name__ == '__main__':
    # read params from command line
    parser = argparse.ArgumentParser(description='Adding models in gazebo world files.')
    parser.add_argument('--world', type=str, help='path to existing world')
    parser.add_argument('--new_world', type=str, help='path of new world')

    parser.add_argument('--model', type=str, help='path to the model')
    parser.add_argument('--model_count', type=int, help='how many models')
    parser.add_argument('--cur_config', type=str, help='config file for properties of existing models in the world')
    parser.add_argument('-w', type=int, help='width of arena')
    parser.add_argument('-l', type=int, help='length of arena')
    parser.add_argument('--thresh_dist', type=float, help='threhold safety distance from an object')
    parser.add_argument('--uav_count', type=int, help='UAVs to be simulated')

    #if using via launch those two params are passed by ros automatically, so you don't need to specify them
    # parser.add_argument('__name', metavar='n', type=str, nargs=1,help='ros param')
    # parser.add_argument('__log', metavar='l', type=str, nargs=1,help='ros param')

    args, unk = parser.parse_known_args()

    # print(args)

    createWorld(org_world = args.world, 
                new_world = args.new_world,
                model = args.model,
                arena_w = args.w,
                arena_l = args.l,
                model_count = args.model_count,
                cur_config = args.cur_config,
                uav_count = args.uav_count,
                thresh_dist = args.thresh_dist)
