#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


from lib.mgeo.class_defs import *
# from d_dijkstra_no_lc import Dijkstra
from e_dijkstra import Dijkstra
from math import sqrt

class globalPathPlanning :

    def __init__(self):
        rospy.init_node('global_path_planner', anonymous=True)
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher('global_path',Path, queue_size=1)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/odom', Odometry, self.current_pose_callback)

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        self.global_planner=Dijkstra(self.nodes,self.links)

        self.is_pose=False

        print('# of nodes: ', len(node_set.nodes))
        print('# of links: ', len(link_set.lines))


        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
   
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
                
            rate.sleep()


 
    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node

    def current_pose_callback(self,msg):
        self.pose_msg=msg
        self.is_pose=True


    def goal_callback(self,msg):
        if self.is_pose==True :
            x=self.pose_msg.pose.pose.position.x
            y=self.pose_msg.pose.pose.position.y
            goal_x=msg.pose.position.x
            goal_y=msg.pose.position.y
            start_min_dis=float('inf')
            goal_min_dis=float('inf')
            for node_idx in self.nodes:
                node_pose_x=self.nodes[node_idx].point[0]
                node_pose_y=self.nodes[node_idx].point[1]
                start_dis=sqrt(pow(x-node_pose_x,2)+pow(y-node_pose_y,2))
                goal_dis=sqrt(pow(goal_x-node_pose_x,2)+pow(goal_y-node_pose_y,2))
                if start_dis < start_min_dis :
                    start_min_dis=start_dis
                    start_node_idx=node_idx
                if goal_dis < goal_min_dis :
                    goal_min_dis=goal_dis
                    end_node_idx=node_idx
  

            
            result, path = self.global_planner.find_shortest_path(start_node_idx, end_node_idx)
            dijkstra_path=Path()
            dijkstra_path.header.frame_id='map'
            for waypoint in path["point_path"] :
                tmp_point=PoseStamped()
                tmp_point.pose.position.x=waypoint[0]
                tmp_point.pose.position.y=waypoint[1]
                dijkstra_path.poses.append(tmp_point)
                print(waypoint[0],waypoint[1])

            
            
            self.global_path_pub.publish(dijkstra_path)


if __name__ == '__main__':
    
    test_track=globalPathPlanning()


