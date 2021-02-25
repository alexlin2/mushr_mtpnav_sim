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

RADIUS = 1.5

init_pose = {} 

current_pose = {}

def get_start_pose(pub_init_pose, plan):
    init_pose["car1"] = plan.pop(0)
    init_pose["car2"] = plan.pop(0)
    #init_3 = plan.pop(0)
    #init_4 = plan.pop(0)
    send_init_pose(pub_init_pose[0], init_pose["car1"])
    send_init_pose(pub_init_pose[1], init_pose["car2"])
    #send_init_pose(pub_init_pose[2], init_3)
    #send_init_pose(pub_init_pose[3], init_4)


def send_nav_goal(plan):
    for c in plan:
        goal = c.split(',')
        car_number = int(goal[0])

        waypoints = []    
        paths = XYHVPath()

        curr_pose_data = current_pose["car"+str(car_number)]

        #waypoints.append((curr_pose_data.position.x, curr_pose_data.position.y))
        
        for i in range(1, len(goal)):
            s = goal[i]
            wp_x = float(s[s.find('(')+1:s.find(' ')])
            wp_y = float(s[s.find(' ')+1:s.find(')')])
            waypoints.append((wp_x, wp_y))
            
        #print(waypoints)

        def dist(x1, x2):
            return math.sqrt( ((x1[0]-x2[0])**2) + ((x1[1]-x2[1])**2) )

        for wp_num in range(len(waypoints)-1):
            x1 = waypoints[wp_num]
            x2 = waypoints[wp_num+1]
    
            d = dist(x1, x2)
            print(d)
            if d < RADIUS:
                path = XYHV()
                path.x = x1[0]
                path.y = x1[1]
                path.h = math.atan2(x2[1] - path.y, x2[0] - path.x)
                path.v = 0.8
                paths.waypoints.append(path) 
            else:
                path_x = np.linspace(x1[0], x2[0], int(d * 2))
                path_y = np.linspace(x1[1], x2[1], int(d * 2))
                for i in range(int(d * 2)-1):
                    path = XYHV()
                    path.x = path_x[i]
                    path.y = path_y[i]
                    path.h = math.atan2(path_y[i+1] - path.y, path_x[i+1] - path.x)
                    path.v = 0.8
                    paths.waypoints.append(path) 
            
        send_path = rospy.ServiceProxy("/car" + str(car_number) +"/rhcontroller/task/path", FollowPath)
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

if __name__ == "__main__":
    rospy.init_node("mtpnav_sim_init")

    plan_file = rospy.get_param("~plan_file")
    real = rospy.get_param("~real_car") == "true"

    with open(plan_file) as f:
        plan = f.readlines()

    rospy.Subscriber(
        "/" + "car1" + "/" + "car_pose",
        PoseStamped,
        cb_pose,
        callback_args="car1",
        queue_size=10,
    )

    rospy.Subscriber(
        "/" + "car2" + "/" + "car_pose",
        PoseStamped,
        cb_pose,
        callback_args="car2",
        queue_size=10,
    )

    if not real:
       
        pub_init_pose_1 = rospy.Publisher("/car1/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose_2 = rospy.Publisher("/car2/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose = [pub_init_pose_1, pub_init_pose_2]

        rospy.sleep(1.0)
        
        get_start_pose(pub_init_pose, plan)


    
    else:
        plan.pop(0)
        plan.pop(0)

    rospy.sleep(1.0)
    
    send_nav_goal(plan)





