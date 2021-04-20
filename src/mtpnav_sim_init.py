#!/usr/bin/env python
import rospy
import numpy as np
import math
import pandas as pd
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

def get_start_pose_2(pub_init_pose, plan):
    send_init_pose_2(pub_init_pose[0], plan[0][0])
    send_init_pose_2(pub_init_pose[1], plan[1][0])

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
        
            
        send_path = rospy.ServiceProxy("/car" + str(car_number) +"/rhcontroller/task/path", FollowPath)
        send_path(paths)
        

def send_nav_goal_2(plan):
    car_nos = ["30", "38"]
    for i in range(len(plan)):
        ref_traj = plan[i]
        paths = XYHVPath()
        for j in range(0, len(ref_traj)):
            path = XYHV()
            wp = ref_traj[j]
            path.x = wp[0]
            path.y = wp[1]
            path.h = wp[2]
            path.v = wp[3]
            paths.waypoints.append(path)

        send_path = rospy.ServiceProxy("/car" + car_nos[i] +"/rhcontroller/task/path", FollowPath)
        send_path(paths)
    
def cb_pose(msg, arg):
    current_pose[arg] = msg.pose
    

def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def send_init_pose_2(pub_init_pose, init_pose):
    x, y, theta = init_pose[0], init_pose[1], init_pose[2]
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def read_csv(plan_file):
    """
    Prepare data for plotting.

    Returns:
    all_ep_data [[[player_1_ep1], [player_2_ep1]], [[player_1_ep2], [player_2_ep2]]...]
    player_1_ep1 [[x], [y], [heading], [vel], [throttle]]
    """
    df = pd.read_csv(plan_file)
    all_ep_data = []
    for i in range(len(df.values)):
        all_player_data = []
        for j in range(2):
            x_coord = [float(idx) for idx in df['x{}'.format(j+1)][i].strip('][').split(', ')]
            y_coord = [float(idx) for idx in df['y{}'.format(j+1)][i].strip('][').split(', ')]
            # TODO: Transform heading angle according to ROS
            heading = [float(idx) for idx in df['h{}'.format(j+1)][i].strip('][').split(', ')]
            vel = [float(idx) for idx in df['v{}'.format(j+1)][i].strip('][').split(', ')]
            throttle = [float(idx) for idx in df['a{}'.format(j+1)][i].strip('][').split(', ')]
            player_data = [x_coord, y_coord, heading, vel, throttle]
            all_player_data.append(player_data)
        all_ep_data.append(all_player_data)

    return all_ep_data

if __name__ == "__main__":
    rospy.init_node("mtpnav_sim_init")

    plan_file = rospy.get_param("~plan_file")
    real = rospy.get_param("~real_car")
    use_algames = rospy.get_param("~use_algames")

    if use_algames:
        data = read_csv(plan_file)
        plan = data[0]
    else:
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
        
        get_start_pose_2(pub_init_pose, plan)
    
    else:
        plan.pop(0)
        plan.pop(0)

    rospy.sleep(1.0)
    
    send_nav_goal_2(plan)





