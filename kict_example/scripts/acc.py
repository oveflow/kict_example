#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os,sys
import rospy
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import PoseStamped,Point32
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Float64
import numpy as np
from factor import *
from morai_msgs.msg import ObjectStatus,ObjectStatusList,EgoVehicleStatus
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from factor import ACC_OBJECT, ACC_PEDESTRIAN, DEFAULT_SPACE ,CAR_MAX_SPEED ,ROAD_FRICTION


class localPathPlanning :

    def __init__(self):
        rospy.init_node('localPathPlanning', anonymous=True)
        self.local_path_pub = rospy.Publisher('local_path',Path, queue_size=1)
        self.target_vel_pub = rospy.Publisher('target_vel',Float64, queue_size=1)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic",ObjectStatusList, self.obj_callback)
        rospy.Subscriber('/odom', Odometry, self.current_pose_callback)
        rospy.Subscriber('/global_path', Path, self.path_callback)
 
        self.local_path_point_num=100
        self.is_pose=False
        self.is_path=False
        self.is_obj=False
        self.is_current_vel=False

        self.target_vel_msg=Float64()
        self.vel_planning=velocityPlanning( CAR_MAX_SPEED / 3.6, ROAD_FRICTION )
        
        vaild_obj=vaildObject()
        cc=cruiseControl(0.5,1) 

        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.is_path and self.is_pose and self.is_obj==True and self.is_current_vel==True :
         
                local_path=Path()
                current_waypoint=-1
                min_dis=float('inf')
                for i in range(len(self.path_msg.poses)) :

                    dx=self.x - self.path_msg.poses[i].pose.position.x
                    dy=self.y - self.path_msg.poses[i].pose.position.y
                    dis=sqrt(dx*dx + dy*dy)
                    if dis < min_dis :
                        min_dis=dis
                        current_waypoint=i


                if current_waypoint+self.local_path_point_num < len(self.path_msg.poses) :
                    end_waypoint= current_waypoint+self.local_path_point_num 

                else :
                    end_waypoint=len(self.path_msg.poses)


                local_path.header.frame_id='map'
                for i in range(current_waypoint,end_waypoint) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=self.path_msg.poses[i].pose.position.x
                    tmp_pose.pose.position.y=self.path_msg.poses[i].pose.position.y
                    tmp_pose.pose.position.z=self.path_msg.poses[i].pose.position.z
                    tmp_pose.pose.orientation.x=0
                    tmp_pose.pose.orientation.y=0
                    tmp_pose.pose.orientation.z=0
                    tmp_pose.pose.orientation.w=1
                    local_path.poses.append(tmp_pose)


                global_vaild_object,local_vaild_object=vaild_obj.calc_vaild_obj(self.object_info_msg,[self.x,self.y,self.vehicle_yaw])
                cc.checkObject(local_path,global_vaild_object,local_vaild_object)
                
        
                target_velocity = cc.acc(local_vaild_object,self.current_vel,self.vel_profile[current_waypoint]) 
      
                self.target_vel_msg.data=target_velocity
                print(target_velocity)


                self.local_path_pub.publish(local_path)
                self.target_vel_pub.publish(self.target_vel_msg)
            
            rate.sleep()


    def current_pose_callback(self,msg):
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.is_pose=True

    def path_callback(self,msg):
        self.path_msg=msg
        self.vel_profile=self.vel_planning.curveBasedVelocity(self.path_msg,50)
        self.is_path=True
    

    def obj_callback(self,msg):
        self.object_info_msg=msg
        self.is_obj=True

    def status_callback(self,msg):  
        self.is_current_vel=True
        self.current_vel=msg.velocity.x

class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
 

    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)


        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            
            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)


        for i in range(len(global_path.poses)-point_num,len(global_path.poses)-10):
            out_vel_plan.append(self.car_max_speed)

        for i in range(len(global_path.poses)-10,len(global_path.poses)):
            out_vel_plan.append(0)
        return out_vel_plan



class vaildObject :


    def calc_vaild_obj(self,obj_msg,ego_pose):
        global_object_info=[]
        loal_object_info=[]
        self.all_object=obj_msg

        tmp_theta=ego_pose[2]
        tmp_translation=[ego_pose[0],ego_pose[1]]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        if self.all_object.num_of_npcs > 0:


            for num in range(self.all_object.num_of_npcs):
                global_result=np.array([[self.all_object.npc_list[num].position.x],[self.all_object.npc_list[num].position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([1,self.all_object.npc_list[num].position.x,self.all_object.npc_list[num].position.y,self.all_object.npc_list[num].velocity/3.6])
                    loal_object_info.append([1,local_result[0][0],local_result[1][0],self.all_object.npc_list[num].velocity/3.6])
            

        if self.all_object.num_of_obstacle > 0:

            for num in range(self.all_object.num_of_obstacle):
                global_result=np.array([[self.all_object.obstacle_list[num].position.x],[self.all_object.obstacle_list[num].position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([2,self.all_object.obstacle_list[num].position.x,self.all_object.obstacle_list[num].position.y,self.all_object.obstacle_list[num].velocity/3.6])
                    loal_object_info.append([2,local_result[0][0],local_result[1][0],self.all_object.obstacle_list[num].velocity/3.6])

        if self.all_object.num_of_pedestrian > 0:

            for num in range(self.all_object.num_of_pedestrian):
                global_result=np.array([[self.all_object.pedestrian_list[num].position.x],[self.all_object.pedestrian_list[num].position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]>0 :
                    global_object_info.append([0,self.all_object.pedestrian_list[num].position.x,self.all_object.pedestrian_list[num].position.y,self.all_object.pedestrian_list[num].velocity/3.6])
                    loal_object_info.append([0,local_result[0][0],local_result[1][0],self.all_object.pedestrian_list[num].velocity/3.6])

        return global_object_info,loal_object_info


class cruiseControl: ## ACC(advanced cruise control) 적용 ##
    def __init__(self,object_vel_gain,object_dis_gain):
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object,tl=[]): ## 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호) ##
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')
            for i in range(len(global_vaild_object)):
                for path in ref_path.poses :      

                    if ACC_OBJECT:

                        if global_vaild_object[i][0]==1 or global_vaild_object[i][0]==2 :    # 차량, 정적장애물 

                            dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                            if dis<2.5:
                                rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.object=[True,i]
                    

                    if ACC_PEDESTRIAN:

                        if global_vaild_object[i][0]==0 :   # 사람
                        
                            dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))

                            if dis<4.35:
                                
                                rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.Person=[True,i]


                    # if global_vaild_object[i][0]==3 :
                    #     traffic_sign='STOP'

                    #     if len(tl)!=0  and  global_vaild_object[i][3] == tl[0] :
                    #         if tl[1] == 48 or tl[1]==16   :   # 신호등 값
                    #             traffic_sign ='GO'
                    #     if traffic_sign =='STOP':
                    #         dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                            
                    #         if dis<9 :
                    #             rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                    #             if rel_distance < min_rel_distance:
                    #                 min_rel_distance=rel_distance
                    #                 self.traffic=[True,i]

                         
                    

    



    def acc(self,local_vaild_object,ego_vel,target_vel): ## advanced cruise control 를 이용한 속도 계획 ##
        out_vel=target_vel
        pre_out_vel = out_vel
        if self.object[0] == True :
            print("ACC ON_vehicle")   
            front_vehicle=[local_vaild_object[self.object[1]][1],local_vaild_object[self.object[1]][2],local_vaild_object[self.object[1]][3]]
            time_gap=0.8
            default_space=5
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
            vel_rel=(front_vehicle[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel

      


        # if self.Person[0]==True:
        #     print("ACC ON_person")
        #     Pedestrian=[local_vaild_object[self.Person[1]][1],local_vaild_object[self.Person[1]][2],local_vaild_object[self.Person[1]][3]]
        #     time_gap=0.8
        #     default_space=8
        #     dis_safe=ego_vel* time_gap+default_space
        #     dis_rel=sqrt(pow(Pedestrian[0],2)+pow(Pedestrian[1],2))-3
            
        #     vel_rel=(Pedestrian[2]-ego_vel)  
            
        #     v_gain=self.object_vel_gain
        #     x_errgain=self.object_dis_gain
        #     acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

        #     acc_based_vel=ego_vel+acceleration
            
        #     if acc_based_vel > target_vel : 
        #         acc_based_vel=target_vel
            
        #     if dis_safe-dis_rel >0 :
        #         out_vel=acc_based_vel - 5
        #     else :
        #         if acc_based_vel<target_vel :
        #             out_vel=acc_based_vel
        #     dx =  Pedestrian[0]
        #     dy =  Pedestrian[1]

        #     t_dis = sqrt(pow(dx,2)+pow(dy,2))


        # if self.traffic[0] == True :
        #     print("Traffic_ON")   
        #     front_vehicle=[local_vaild_object[self.traffic[1]][1],local_vaild_object[self.traffic[1]][2],local_vaild_object[self.traffic[1]][3]]
        #     time_gap=0.8
        #     default_space=3
        #     dis_safe=ego_vel* time_gap+default_space
        #     dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-3
            
        #     vel_rel=(0-ego_vel)  
            
        #     v_gain=self.object_vel_gain
        #     x_errgain=self.object_dis_gain
        #     acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

        #     acc_based_vel=ego_vel+acceleration
            
        #     if acc_based_vel > target_vel : 
        #         acc_based_vel=target_vel
            
        #     if dis_safe-dis_rel >0 :
        #         out_vel=acc_based_vel
        #     else :
        #         if acc_based_vel<target_vel :
        #             out_vel=acc_based_vel

        #     if dis_rel < 3 :
        #         out_vel = 0
        return out_vel



if __name__ == '__main__':
    
    test=localPathPlanning()