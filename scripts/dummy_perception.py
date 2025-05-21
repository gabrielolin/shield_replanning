import numpy as np
import rospy
import os
import sys
from shield_planner_msgs.msg import Projectile
from dm_control.utils.inverse_kinematics import qpos_from_site_pose
from dm_control.utils.transformations import euler_to_quat
from dm_control import mujoco as dm_mj
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../mujoco_ws/scripts/')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../mujoco_ws/scripts/uncertainty/')))
from testbed_compute_projectile import computeParabolaEqn
from projectile_test import get_qpos_from_proj, getSiteTransformWRTWorld
from sensor_msgs.msg import JointState

projectile = None
new_msg_received = False
def proj_cb(msg):
    global projectile
    global new_msg_received
    projectile = np.array([
        msg.position.x + T_W_Base[0],
        msg.position.y + T_W_Base[1],
        msg.position.z + T_W_Base[2],
        msg.velocity.x,
        msg.velocity.y,
        msg.velocity.z
    ], dtype=float)
    new_msg_received = True
    rospy.loginfo(f"Received projectile: {projectile}")

rospy.init_node('mujoco_ik_node')
joint_pub = rospy.Publisher('/mujoco_ik', JointState, queue_size=10)
proj_sub = rospy.Subscriber('/projectile', Projectile, proj_cb)
model_dir = '/home/shield/code/shield_min_ws/src/replanning/mujoco_ws/abb/irb_1600/'
mjcf = 'irb1600_6_12_shield_projectile.xml'
dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))

T_W_Base =np.array([0.55, -0.7, 0.45])

rate = rospy.Rate(10)  
#new_msg_received = True
#projectile = np.array([6.955, -0.19, 1.29246452, -10, 0, 2.52799])
#projectile2 = np.array([6.955, -0.19, 1.59246452, -10, 0, 2.72799])
while not rospy.is_shutdown():
    if not new_msg_received:
        rate.sleep()
        continue

    try:
        qpos, success = get_qpos_from_proj(projectile, dm_model)
        if not success:
            rospy.logwarn("IK Failed")
        else:
            msg = JointState()
            msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            msg.position = qpos.tolist()
            msg.header.stamp = rospy.Time.now()
            joint_pub.publish(msg)
            rospy.loginfo(f"Published IK qpos: {qpos}")
    except Exception as e:
        rospy.logwarn(f"Failed to compute IK: {e}")

    new_msg_received = False  # Reset flag after processing
    rate.sleep()



