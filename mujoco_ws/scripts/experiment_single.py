import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pdb

import time
import os
import rospy
import std_msgs.msg
import sys
from shield_planner_msgs.msg import Projectile
from shield_planner_msgs.srv import PlanPathProjectile

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),"camera_calibration"))

def main():
    # Read ind the projectiles from files
    proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../logs/animation_projs.txt")
    # proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../logs/rand_projs.txt")

    traj_mat = np.loadtxt(proj_path, dtype=float)

    # Init ROS node
    rospy.init_node('experiment_sim')
    rospy.wait_for_service('plan_path_projectile')

    # Init a client
    planner = rospy.ServiceProxy('plan_path_projectile', PlanPathProjectile)

    # Init a publisher
    projectile_msg_pub = rospy.Publisher("projectile", 
                                        Projectile,
                                        queue_size=1)
    
    rospy.sleep(1)

    count = 0
    for i in range(traj_mat.shape[0]):
        # Init a projectile
    # Single traj picking for reproducable experiments
    # i = 5
    # if 1:
        projectile_msg = Projectile()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'odom_combined'
        projectile_msg.header = header
        projectile_msg.object_id = 0

        projectile_msg.position.x = traj_mat[i][0]
        projectile_msg.position.y = traj_mat[i][1]
        projectile_msg.position.z = traj_mat[i][2]
        projectile_msg.velocity.x = traj_mat[i][3]
        projectile_msg.velocity.y = traj_mat[i][4]
        projectile_msg.velocity.z = traj_mat[i][5]

        projectile_msg_pub.publish(projectile_msg)
        time.sleep(20)
    # Invoke the service, this part no longer needed. Since we have executive running
    # try:
    #     resp = planner(projectile_msg, 0, 0)
    # except rospy.ServiceException as exc:
    #     count = count + 1
    #     print("Fail # {}".format(count))

    exit()


if __name__ == "__main__":
    main()