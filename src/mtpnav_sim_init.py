#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    PoseStamped,
    Quaternion,
)
from mushr_rhc_ros.srv import FollowPath
from mushr_rhc_ros.msg import XYHVPath, XYHV
from tf.transformations import quaternion_from_euler

init_pose = {} 

def get_start_pose(pub_init_pose, plan):
    init_pose["car30"] = plan.pop(0)
    init_pose["car38"] = plan.pop(0)
    #init_3 = plan.pop(0)
    #init_4 = plan.pop(0)
    send_init_pose(pub_init_pose[0], init_pose["car30"])
    send_init_pose(pub_init_pose[1], init_pose["car38"])
    #send_init_pose(pub_init_pose[2], init_3)
    #send_init_pose(pub_init_pose[3], init_4)


def send_nav_goal(plan):
    for c in plan:
        goal = c.split(',')
        car_number = int(goal[0])

        waypoints = []    
        paths = XYHVPath()
        
        for i in range(1, len(goal)):
            s = goal[i][goal[i].find('(')+1:goal[i].find(')')].split()
            wp_x = float(s[0])
            wp_y = float(s[1])
            wp_v = float(s[2])
            waypoints.append((wp_x, wp_y, wp_v))
            
        #print(waypoints)

        for wp_num in range(len(waypoints)-1):
            x1 = waypoints[wp_num]
            x2 = waypoints[wp_num+1]
            path = XYHV()
            path.x = x1[0]
            path.y = x1[1]
            path.h = math.atan2(x2[1] - path.y, x2[0] - path.x)
            path.v = x1[2]
            paths.waypoints.append(path) 
        
        if car_number == 30:
            send_path = rospy.ServiceProxy("/test/task/path", FollowPath)
            send_path(paths)

        else:
            send_path = rospy.ServiceProxy("/car" + str(car_number) +"/rhcontroller/task/path", FollowPath)
            send_path(paths)
    return paths
        
    
def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

if __name__ == "__main__":
    rospy.init_node("mtpnav_sim_init")

    plan_file = rospy.get_param("~plan_file")
    real = rospy.get_param("~real_car")

    with open(plan_file) as f:
        plan = f.readlines()

    if not real:
       
        pub_init_pose_1 = rospy.Publisher("/car30/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose_2 = rospy.Publisher("/car38/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose = [pub_init_pose_1, pub_init_pose_2]
        rospy.loginfo("Currently in sim")
        rospy.sleep(1.0)
        
        get_start_pose(pub_init_pose, plan)


    
    else:
        plan.pop(0)
        plan.pop(0)

    rospy.sleep(1.0)
    
    send_nav_goal(plan)
    



