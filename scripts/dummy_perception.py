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
from projectile_test import get_qpos_from_proj
from sensor_msgs.msg import JointState

rospy.init_node('mujoco_ik_node')
joint_pub = rospy.Publisher('/mujoco_ik', JointState, queue_size=10)

model_dir = '/home/shield/code/shield_min_ws/src/replanning/mujoco_ws/abb/irb_1600/'
mjcf = 'irb1600_6_12_shield_projectile.xml'
dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))

rate = rospy.Rate(10)  # 1 Hz

# Replace with your real projectile data
projectile = np.array([6.955, -0.19, 1.29246452, -10, 0, 2.52799])
projectile2 = np.array([6.955, -0.19, 1.59246452, -10, 0, 2.72799])
while not rospy.is_shutdown():
    try:
        qpos = get_qpos_from_proj(projectile, dm_model)
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = qpos.tolist()
        msg.header.stamp =  rospy.Time.now()
        joint_pub.publish(msg)
        rospy.loginfo(f"Published IK qpos: {qpos}")
    except Exception as e:
        rospy.logwarn(f"Failed to compute IK: {e}")

    rate.sleep()




