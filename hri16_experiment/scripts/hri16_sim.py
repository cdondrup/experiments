#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 11 16:10:43 2015

@author: cdondrup
"""

import rospy
from test_cases.srv import Load, LoadRequest, Run
from std_srvs.srv import Empty
from dynamic_reconfigure.client import Client as DynClient
import csv
from geometry_msgs.msg import Pose, PoseStamped
from bayes_people_tracker.msg import PeopleTracker
import numpy as np
from hrsi_representation.msg import QTCArray
import json
from hrsi_state_prediction.srv import LoadModel, LoadModelRequest


class Test(object):
    __no_state__ = 9.

    __config_lookup = [
        {"distance_threshold": 6.0},
        {"distance_threshold": 4.0},
        {"distance_threshold": 4.0}
    ]

    __scenario_lookup = [
        "passby_corridor_hard",
        "path_crossing_right",
        "corner_crossing"
    ]

    __model_paths = [
        "/home/cdondrup/tmp/hri/passby/qtcc.model",
        "/home/cdondrup/tmp/hri/pathcrossing/qtcc.model",
        "/home/cdondrup/tmp/hri/corner/qtcbc.model"
    ]

    __qtc_buffer = {}


    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.num_trials = rospy.get_param("~num_trials")
        self.interaction_type = {"type": rospy.get_param("~interaction_type")}
        self.scenario = self.__scenario_lookup[self.interaction_type["type"]]
        self.model = self.__model_paths[self.interaction_type["type"]]
        self.out_dir = rospy.get_param("~out_dir")

        self.pred_dyn = DynClient("qtc_state_predictor")
        self.pred_dyn.update_configuration(self.interaction_type)

        self.crea_dyn = DynClient("online_qtc_creator")
        self.crea_dyn.update_configuration(self.__config_lookup[self.interaction_type["type"]])

        self.ppl_topic = rospy.get_param("~ppl_topic", "/people_tracker/positions")
        self.robot_topic = rospy.get_param("~robot_topic", "/robot_pose")
        self.qtc_topic = rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose_republisher/pose")

        self.robot_pose = None
        self.goal_pose = PoseStamped()

        rospy.loginfo("... loading model")
        try:
            s = rospy.ServiceProxy("/qtc_state_predictor/load_model", LoadModel)
            rospy.loginfo("... waiting for service %s" % s.resolved_name)
            s.wait_for_service()
            rospy.loginfo("... loading model %s" % self.model)
            l = LoadModelRequest(filename=self.model)
            s(l)
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            rospy.logfatal(e)

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

        self.trajectories = []

        rospy.loginfo("... done")

        rospy.loginfo("... all done")

    def ppl_callback(self, msg):
        if self.robot_pose == None:
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


    def pose_callback(self, msg):
        self.robot_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg

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

#            new = self.get_new_states(elem.uuid, qtc, self.__qtc_buffer[elem.uuid]["robot"])
#            if new != None:
#                self.__qtc_buffer[elem.uuid]["robot"] = np.append(
#                    self.__qtc_buffer[elem.uuid]["robot"],
#                    new
#                ).reshape(-1, 4)


#            new = self.get_new_states(elem.uuid, qtc, self.__qtc_buffer[elem.uuid]["goal"])
#            if new != None:
#                self.__qtc_buffer[elem.uuid]["goal"] = np.append(
#                    self.__qtc_buffer[elem.uuid]["goal"],
#                    new
#                ).reshape(-1, 4)

    def run_test(self):
        ret = []
        i = 0
        while not rospy.is_shutdown() and i < self.num_trials:
            rospy.loginfo("Starting run %s/%s" % (i+1, self.num_trials))
            self.trajectories.append([])
            rospy.loginfo("Creating services ...")
            try:
                self.crea_dyn.update_configuration({"decay_time": .1})
                r = rospy.ServiceProxy("/people_tracker/new_id", Empty)
                rospy.loginfo("  ... waiting for %s" % r.resolved_name)
                r.wait_for_service()
                rospy.loginfo("  ... calling %s" % r.resolved_name)
                r()
                rospy.loginfo("  ... done")

                r = rospy.ServiceProxy("/scenario_server/reset", Empty)
                rospy.loginfo("  ... waiting for %s" % r.resolved_name)
                r.wait_for_service()
                rospy.loginfo("  ... calling %s" % r.resolved_name)
                r()
                rospy.loginfo("  ... done")

                r = rospy.ServiceProxy("/people_tracker/new_id", Empty)
                rospy.loginfo("  ... waiting for %s" % r.resolved_name)
                r.wait_for_service()
                rospy.loginfo("  ... calling %s" % r.resolved_name)
                r()
                rospy.loginfo("  ... done")

                rospy.loginfo("Subscribing to human and robot pose")
                ps = rospy.Subscriber(
                    self.ppl_topic,
                    PeopleTracker,
                    callback=self.ppl_callback,
                    queue_size=1
                )
                rs = rospy.Subscriber(
                    self.robot_topic,
                    Pose,
                    callback=self.pose_callback,
                    queue_size=1
                )
                qs = rospy.Subscriber(
                    self.qtc_topic,
                    QTCArray,
                    callback=self.qtc_callback,
                    queue_size=10
                )
                gs = rospy.Subscriber(
                    self.goal_topic,
                    PoseStamped,
                    callback=self.goal_callback,
                    queue_size=1
                )

                self.crea_dyn.update_configuration({"decay_time": 2.})

                s = rospy.ServiceProxy("/scenario_server/start", Run)
                rospy.loginfo("  ... waiting for %s" % s.resolved_name)
                s.wait_for_service()
                rospy.loginfo("  ... calling %s" % s.resolved_name)
                result = s()
                rospy.loginfo("  ... done")
                d = {}
                d["nav_success"] = result.nav_success
                d["human_success"] = result.human_success
                d["min_distance_to_human"] = result.min_distance_to_human
                d["mean_speed"] = result.mean_speed
                d["distance_travelled"] = result.distance_travelled
                d["travel_time"] = result.travel_time
                ret.append(d)
            except (rospy.ServiceException, rospy.ROSInterruptException) as e:
                rospy.logfatal(e)
            finally:
                rospy.loginfo("Unsubscribing")
                ps.unregister()
                rs.unregister()
                qs.unregister()
                gs.unregister()
                ps = None; rs = None; qs = None; gs = None

            i += 1

        return ret, self.trajectories

    def write_file(self, stats, trajectories):
        rospy.loginfo("Writing results to %s" % self.out_dir)
        with open(self.out_dir+"/stats.csv", 'w') as f:
            rospy.loginfo("Writing stats.csv")
            writer = csv.DictWriter(f, stats[0].keys())
            writer.writeheader()
            writer.writerows(stats)

        for i, t in enumerate(trajectories):
            name = str(i)+".csv"
            with open(self.out_dir+"/"+name, 'w') as f:
                rospy.loginfo("Writing %s" % name)
                writer = csv.DictWriter(f, t[0].keys())
                writer.writeheader()
                writer.writerows(t)

        for i, v in enumerate(self.__qtc_buffer.values()):
            name = str(i)+"_qtc.txt"
            with open(self.out_dir+"/"+name, 'w') as f:
                rospy.loginfo("Writing %s" % name)
                np.savetxt(f, v, fmt='%.0f')
#            name = str(i)+"_qtc.txt"
#            with open(self.out_dir+"/"+name, 'w') as f:
#                rospy.loginfo("Writing %s" % name)
#                np.savetxt(f, v["goal"], fmt='%.0f')


if __name__ == "__main__":
    rospy.init_node("test_icra16")
    t = Test(rospy.get_name())
    r, tra = t.run_test()
    t.write_file(r, tra)
#    rospy.spin()
