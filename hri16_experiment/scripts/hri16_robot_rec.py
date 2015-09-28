#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 11 16:10:43 2015

@author: cdondrup
"""

import rospy
from dynamic_reconfigure.client import Client as DynClient
import csv
from geometry_msgs.msg import Pose, PoseStamped
from bayes_people_tracker.msg import PeopleTracker
import numpy as np
from hrsi_representation.msg import QTCArray
import json
from scitos_teleop.msg import action_buttons
from std_srvs.srv import Empty, EmptyResponse
from test_cases.srv import Load, LoadRequest
from fake_camera_effects.msg import CameraEffectsAction, CameraEffectsGoal
from actionlib import SimpleActionClient
import yaml
from roslib.packages import find_resource
from collections import OrderedDict
import os


PKG = "hri16_experiment"


class Test(object):
    __no_state__ = 9.

    __config_lookup = [
        {"distance_threshold": 4.0},
        {"distance_threshold": 4.0},
        {"distance_threshold": 4.0}
    ]

    __qtc_buffer = OrderedDict()


    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.out_dir = rospy.get_param("~out_dir")
        self.par = str(rospy.get_param("~par"))

        self.client = SimpleActionClient("/camera_effects", CameraEffectsAction)
        self.client.wait_for_server()

        self.crea_dyn = DynClient("online_qtc_creator")
        self.crea_dyn.update_configuration(self.__config_lookup[0])

        with open(find_resource(PKG, 'fake_goals.yaml')[0],'r') as f:
            conf = yaml.load(f)

        self.ppl_topic = rospy.get_param("~ppl_topic", "/people_tracker/positions")
        self.robot_topic = rospy.get_param("~robot_topic", "/robot_pose")
        self.qtc_topic = rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array")
#        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose_republisher/pose")

        rospy.Service("~save", Empty, self.write_file)

        self.robot_pose = None
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = conf["point"]["x"]
        self.goal_pose.pose.position.y = conf["point"]["y"]

        self.pub = rospy.Publisher("/fake_goal", PoseStamped, queue_size=1, latch=True)
        self.pub.publish(self.goal_pose)

        self.num_trial = 0
        self.scenario = "record_robot"

        self.trajectories = []
        self.traj_name = []
        self.ret = []

        rospy.loginfo("... loading scenario")
        try:
            s = rospy.ServiceProxy("/scenario_server/load", Load)
            rospy.loginfo("... waiting for service")
            s.wait_for_service()
            rospy.loginfo("... loading scenario %s" % self.scenario)
            l = LoadRequest(scenario=self.scenario)
            s(l)
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logfatal(e)

        rospy.Subscriber(
            "/teleop_joystick/action_buttons",
            action_buttons,
            self.button_callback,
            queue_size=1
        )

        rospy.loginfo("... done")

        rospy.loginfo("... all done")

    def ppl_callback(self, msg):
        if self.robot_pose == None or not msg.poses:
            return
        msgs = {
            "agent1": "robot",
            "agent2": "human",
            "agent3": "goal",
            "x1": self.robot_pose.position.x,
            "y1": self.robot_pose.position.y,
            "x2": msg.poses[0].position.x,
            "y2": msg.poses[0].position.y,
            "x3": self.goal_pose.pose.position.x,
            "y3": self.goal_pose.pose.position.y
        }
        self.trajectories[-1].append(msgs)
        if not msg.uuids[0] in self.traj_name: self.traj_name.append(msg.uuids[0])


    def pose_callback(self, msg):
        self.robot_pose = msg

#    def goal_callback(self, msg):
#        self.goal_pose = msg

    def get_new_states(self, key, qtc, buf):
        try:
            return qtc[np.where(np.all(qtc==buf[-1], axis=1))[0][-1]+1:]
        except IndexError:
            return qtc

    def qtc_callback(self, msg):
        for elem in msg.qtc:
            if elem.uuid not in self.__qtc_buffer.keys():
                self.__qtc_buffer[elem.uuid] = np.array([])

            qtc = np.array(json.loads(elem.qtc_goal_human))
            qtc[np.isnan(qtc)] = self.__no_state__

            self.__qtc_buffer[elem.uuid] = np.append(
                self.__qtc_buffer[elem.uuid],
                qtc[-1][[1,3]]
            )

            qtc = np.array(json.loads(elem.qtc_robot_human))
            qtc[np.isnan(qtc)] = self.__no_state__

            self.__qtc_buffer[elem.uuid] = np.append(
                self.__qtc_buffer[elem.uuid],
                qtc[-1][[1,3,0,2]]
            ).reshape(-1, 6)

    def button_callback(self, msg):
        rospy.loginfo("Button pressed")
        if msg.A:
            rospy.loginfo("Starting run %s" % self.num_trial)
            self.num_trial += 1
            self.trajectories.append([])
            rospy.loginfo("Creating services ...")
            try:
                rospy.loginfo("Subscribing to human and robot pose")
                self.ps = rospy.Subscriber(
                    self.ppl_topic,
                    PeopleTracker,
                    callback=self.ppl_callback,
                    queue_size=1
                )
                self.rs = rospy.Subscriber(
                    self.robot_topic,
                    Pose,
                    callback=self.pose_callback,
                    queue_size=1
                )
                self.qs = rospy.Subscriber(
                    self.qtc_topic,
                    QTCArray,
                    callback=self.qtc_callback,
                    queue_size=10
                )
#                gs = rospy.Subscriber(
#                    self.goal_topic,
#                    PoseStamped,
#                    callback=self.goal_callback,
#                    queue_size=1
#                )

                self.client.send_goal(CameraEffectsGoal())

            except (rospy.ServiceException, rospy.ROSInterruptException) as e:
                rospy.logfatal(e)
            finally:
                pass

        elif msg.B:
            rospy.loginfo("Unsubscribing")
            try:
                self.ps.unregister()
                self.rs.unregister()
                self.qs.unregister()
    #            gs.unregister()
            except UnboundLocalError as e:
                rospy.logwarn(e)
            self.ps = None; self.rs = None; self.qs = None; #gs = None

            try:
                r = rospy.ServiceProxy("/scenario_server/reset", Empty)
                rospy.loginfo("  ... waiting for %s" % r.resolved_name)
                r.wait_for_service()
                rospy.loginfo("  ... calling %s" % r.resolved_name)
                r()
                rospy.loginfo("  ... done")
            except (rospy.ServiceException, rospy.ROSInterruptException) as e:
                rospy.logfatal(e)

            self.write_file(None)
            self.client.send_goal(CameraEffectsGoal())

    def write_file(self, req):
        trajectories = self.trajectories
        rospy.loginfo("Writing results to %s" % self.out_dir)
        mydir = os.path.join(self.out_dir, "p"+self.par)
        try:
            os.makedirs(mydir)
        except OSError as e:
            rospy.logwarn(e)

        for i, t in enumerate(trajectories):
            name = "p"+self.par+"_"+str(i)+"_"+self.traj_name[i]+".csv"
            with open(mydir+"/"+name, 'w') as f:
                rospy.loginfo("Writing %s" % name)
                try:
                    writer = csv.DictWriter(f, t[0].keys())
                    writer.writeheader()
                    writer.writerows(t)
                except Exception as e:
                    rospy.logwarn(e)

        for i, (k,v) in enumerate(self.__qtc_buffer.items()):
            name = "p"+self.par+"_"+str(i)+"_"+k+"_qtc.txt"
            with open(mydir+"/"+name, 'w') as f:
                rospy.loginfo("Writing %s" % name)
                try:
                    np.savetxt(f, v, fmt='%.0f')
                except Exception as e:
                    rospy.logwarn(e)

        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("hri_record")
    t = Test(rospy.get_name())
    rospy.spin()
