#!/usr/bin/env python3
import rospy
import numpy as np
import math
import torch
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
from scipy.spatial.transform import Rotation as R

init_pose = {}

def get_start_pose(pub_init_pose, plan):
    init_pose["car30"] = plan[0][0][0], plan[0][1][0], plan[0][2][0]
    init_pose["car38"] = plan[1][0][0], plan[1][1][0], plan[1][2][0]
    plan[0][0] = plan[0][0][1:]
    plan[0][1] = plan[0][1][1:]
    plan[0][2] = plan[0][2][1:]
    plan[0][3] = plan[0][3][1:]
    plan[1][0] = plan[1][0][1:]
    plan[1][1] = plan[1][1][1:]
    plan[1][2] = plan[1][2][1:]
    plan[1][3] = plan[1][3][1:]
    #init_3 = plan.pop(0)
    #init_4 = plan.pop(0)
    send_init_pose(pub_init_pose[0], init_pose["car30"])
    send_init_pose(pub_init_pose[1], init_pose["car38"])
    #send_init_pose(pub_init_pose[2], init_3)
    #send_init_pose(pub_init_pose[3], init_4)

def send_nav_goal(plan):
    two_paths = []
    for i in range(len(plan)):
        ref_traj = plan[i]
        paths = XYHVPath()
        for j in np.arange(0, len(ref_traj[0])-1, 2):
            path = XYHV()
            wp = ref_traj[0][j], ref_traj[1][j], ref_traj[3][j]
            wp2 = ref_traj[0][j+1], ref_traj[1][j+1], ref_traj[3][j+1]
            path.x = wp[0]
            path.y = wp[1]
            path.h = math.atan2(wp2[1] - path.y, wp2[0] - path.x)
            path.v = wp[2]
            paths.waypoints.append(path)
        two_paths.append(paths)

    send_path1 = rospy.ServiceProxy("/car30" +"/rhcontroller/task/path", FollowPath)
    send_path2 = rospy.ServiceProxy("/car38" +"/rhcontroller/task/path", FollowPath)
    send_path1(two_paths[0])
    send_path2(two_paths[1])
    

def send_init_pose(pub_init_pose, init_pose):
    x, y, theta = init_pose[0], init_pose[1], init_pose[2]
    r = R.from_euler('z', theta)
    q = Quaternion(r.as_quat()[0],r.as_quat()[1],r.as_quat()[2],r.as_quat()[3])
    point = Point(x=x, y=y)
    print(point)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def read_pt(plan_file, index):
    tensor_data = torch.load(plan_file)
    all_player_data = []
    for i in range(2):
        x_coord = tensor_data[index]['x' + str(i) + '_data']
        y_coord = tensor_data[index]['y' + str(i) + '_data']
        yaw = tensor_data[index]['yaw' + str(i) + '_data']
        speed = tensor_data[index]['speed' + str(i) + '_data']
        all_player_data.append([x_coord, y_coord, yaw, speed])
    return all_player_data

if __name__ == "__main__":
    rospy.init_node("mtpnav_sim_init")

    plan_file = rospy.get_param("~plan_file")
    real = rospy.get_param("~real_car")

    plan = read_pt(plan_file, 55)
    
    if not real:
       
        pub_init_pose_1 = rospy.Publisher("/car30/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose_2 = rospy.Publisher("/car38/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose = [pub_init_pose_1, pub_init_pose_2]
        rospy.loginfo("Currently in sim")
        rospy.sleep(1.0)
        
        get_start_pose(pub_init_pose, plan)

    rospy.sleep(2.0)
    
    send_nav_goal(plan)





