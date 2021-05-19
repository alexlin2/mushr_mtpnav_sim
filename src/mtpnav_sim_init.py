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

NUM_AGENTS = 2

def get_start_pose(pub_init_pose, plan):
    send_init_pose(pub_init_pose[0], plan[0])
    send_init_pose(pub_init_pose[1], plan[1])
        
def send_nav_goal(plan):
    two_paths = []
    for i in range(len(plan)):
        ref_traj = plan[i]
        paths = XYHVPath()
        for j in np.arange(0, len(ref_traj[0])-1, 3):
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
    x, y, theta = init_pose[0][0], init_pose[1][0], init_pose[2][0]
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    print(point)
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
    solved = df['solve'].values
    for i in range(len(df.values)):
        if solved[i]:
            all_player_data = []
            for j in range(NUM_AGENTS):
                x_coord = [float(idx) for idx in df['x{}'.format(j+1)][i].strip('][').split(', ')]
                y_coord = [float(idx) for idx in df['y{}'.format(j+1)][i].strip('][').split(', ')]
                # TODO: Transform heading angle according to ROS
                # For algames, car facing east is at 0 deg and facing north is at 90 deg
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

    data = read_csv(plan_file)

    for plan in data:

        if not real:
        
            pub_init_pose_1 = rospy.Publisher("/car30/initialpose", PoseWithCovarianceStamped, queue_size=1)
            pub_init_pose_2 = rospy.Publisher("/car38/initialpose", PoseWithCovarianceStamped, queue_size=1)
            #pub_init_pose_3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
            #pub_init_pose_4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)
            pub_init_pose = [pub_init_pose_1, pub_init_pose_2]
            rospy.loginfo("Currently in sim")
            rospy.sleep(1.0)
            
            get_start_pose(pub_init_pose, plan)

        rospy.sleep(1.0)
        
        send_nav_goal(plan)

        rospy.sleep(5.0)





