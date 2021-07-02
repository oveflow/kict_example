#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os,sys
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import PointCloud
from factor import PATH_NAME, WAYPOINT_DISTANCE

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class make_path :

    def __init__(self):
        rospy.init_node('make_path', anonymous=True)

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        self.path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.is_odom=False
        self.path_msg=Path()
        self.path_msg.header.frame_id='/map'
        self.prev_x=0
        self.prev_y=0
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('kict_example')
        full_path=pkg_path+'/path/'+PATH_NAME
        self.f=open(full_path,'w')

        self.get_mgeo()
        self.f.close()

    def get_mgeo(self):
        
        self.link_pub = rospy.Publisher('link',PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('node',PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher('global_path',Path, queue_size=1)

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()

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
  
    def odom_callback(self,msg):
        waypint_pose=PoseStamped()
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        if self.is_odom== True :
            distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
            if distance > WAYPOINT_DISTANCE : #(m)
                waypint_pose.pose.position.x=x
                waypint_pose.pose.position.y=y
                waypint_pose.pose.orientation.w=1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data='{0}\t{1}\n'.format(x,y)
                self.f.write(data) 
                self.prev_x=x
                self.prev_y=y
                print(x,y)       
        else :
            self.is_odom=True
            self.prev_x=x
            self.prev_y=y
            
        
if __name__ == '__main__':
    try:
        test_track=make_path()
    except rospy.ROSInterruptException:
        pass

