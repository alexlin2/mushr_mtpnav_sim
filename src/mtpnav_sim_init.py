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

init_pose = {"car1":"", "car2":"", "car3":"", "car4":""} 

current_pose = {"car1":"", "car2":"", "car3":"", "car4":""}

def get_start_pose(pub_init_pose, plan):
    init_pose["car1"] = plan.pop(0)
    #init_pose["car2"] = plan.pop(0)
    #init_3 = plan.pop(0)
    #init_4 = plan.pop(0)
    send_init_pose(pub_init_pose[0], init_pose["car1"])
    #send_init_pose(pub_init_pose[1], init_pose["car2"])
    #send_init_pose(pub_init_pose[2], init_3)
    #send_init_pose(pub_init_pose[3], init_4)


def send_nav_goal(plan):
    for c in plan:
        goal = c.split(',')
        car_number = int(goal[0])
        dest_x = float(goal[1])
        dest_y = float(goal[2])

        pose_data = current_pose["car"+str(car_number)]

        path_x = np.linspace(pose_data.position.x, dest_x, num=20)
        path_y = np.linspace(pose_data.position.y, dest_y, num=20)
        
        paths = XYHVPath()
        for i in range(20):
            path = XYHV()
            path.x = path_x[i]
            path.y = path_y[i]
            path.h = math.atan2(path.y, path.x)
            path.v = float(1)
            paths.waypoints.append(path) 

        send_path = rospy.ServiceProxy("/car" + str(car_number) +"/rhcontroller/task/path", FollowPath)
        send_path(paths)
    

def cb_pose(msg, arg):
    current_pose[arg] = msg.pose
    

def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3
    print(pose_data)

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

    if not real:
       
        pub_init_pose_1 = rospy.Publisher("/car1/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_2 = rospy.Publisher("/car2/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_3 = rospy.Publisher("/car3/initialpose", PoseWithCovarianceStamped, queue_size=1)
        #pub_init_pose_4 = rospy.Publisher("/car4/initialpose", PoseWithCovarianceStamped, queue_size=1)
        pub_init_pose = [pub_init_pose_1]

        rospy.sleep(1.0)
        
        get_start_pose(pub_init_pose, plan)

    
    else:
        plan.pop(0)

    rospy.sleep(1.0)
    
    send_nav_goal(plan)





