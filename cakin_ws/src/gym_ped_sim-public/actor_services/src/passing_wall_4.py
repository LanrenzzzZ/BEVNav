#!/usr/bin/env python
# coding=utf-8

'''
Author:Tai Lei
Date:
Info:
'''

import random
import numpy as np
import rospkg
import rospy
from lxml import etree
from lxml.etree import Element
from copy import deepcopy
import yaml

rospack = rospkg.RosPack()

plugin_pkg_path = rospack.get_path("actor_plugin")
plugin_path = plugin_pkg_path + "/lib/libactorplugin_ros.so"
actor_pkg_path = rospack.get_path("actor_services")

# world_name = rospy.get_param("BASE_WORLD")

# tree_ = etree.parse(actor_pkg_path+'/worlds/'+world_name)
tree_ = etree.parse(actor_pkg_path+'/worlds/' + 'ped_world.world')
world_ = tree_.getroot().getchildren()[0]

skin_list = ["moonwalk.dae",
        "run.dae",
        "sit_down.dae",
        "sitting.dae",
        "stand_up.dae",
        "stand.dae",
        "talk_a.dae",
        "talk_b.dae",
        "walk.dae"]

startingPosition = dict()
targetPosition = dict()
speedOfActor = dict()

startingPosition = {0:(-3,-0.5), 1:(-2.5,0.5), 2:(3, -0.5), 3:(2.5,0.5)}
targetPosition = {0: (2.5,-0.5), 1:(3,0.5),   2:(-2.5, -0.5), 3:(-3,0.5)}
speedOfActor = {0:1.2,1:0.9,2:1.1,3:1.2}
#for item in range(4):
#    speedOfActor[item] = 1.00


actor_list = []
for item in range(4):
    actor = Element("actor", name="actor"+str(item))

    pose = Element("pose")
    #randomly generate position to pose text
    x = str(startingPosition[item][0])
    y = str(startingPosition[item][1])
    pose.text = x+" "+y+" "+"1.02 0 0 0"
    actor.append(pose)

    skin = Element("skin")
    skin_fn = Element("filename")
    skin_fn.text=random.choice(skin_list)
    skin_scale = Element("scale")
    skin_scale.text = "1"
    skin.append(skin_fn)
    skin.append(skin_scale)
    actor.append(skin)

    animation = Element("animation", name="walking")
    animate_fn = Element("filename")
    if (item==int(rospy.get_param("TB3_WITH_ACTOR")[-1])) and (not rospy.get_param("TB3_AS_ACTOR")):
        animate_fn.text = "stand.dae"
    else:
        animate_fn.text = "walk.dae"
    interpolate_x = Element("interpolate_x")
    interpolate_x.text = "true"
    animate_scale = Element("scale")
    animate_scale.text = "1"
    animation.append(animate_fn)
    animation.append(animate_scale)
    animation.append(interpolate_x)
    actor.append(animation)

    plugin = Element("plugin", name="None", filename=plugin_path)
    speed = Element("speed")
    speed.text = str(speedOfActor[item])
    target = Element("target")
    x = str(targetPosition[item][0])
    y = str(targetPosition[item][1])
    target.text =  x+" "+y+" "+"1.02"
    ignore_obstacle = Element("ignore_obstacles")
    model_cafe = Element("model")
    model_cafe.text = "caffe"
    model_ground_plane = Element("model")
    model_ground_plane.text = "ground_plane"
    ignore_obstacle.append(model_cafe)
    ignore_obstacle.append(model_ground_plane)
    plugin.append(speed)
    plugin.append(target)
    plugin.append(ignore_obstacle)
    actor.append(plugin)

    world_.append(actor)

tree_.write(actor_pkg_path+'/worlds/ped_world.world', pretty_print=True, xml_declaration=True, encoding="utf-8")
