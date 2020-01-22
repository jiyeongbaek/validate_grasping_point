import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
from math import radians
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/STEFAN/stl/"
       
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id="base"
        self.assembly_pose.pose.orientation.w = 1.0
        self.assembly_pose.pose.position.x = 1.1
        self.assembly_pose.pose.position.y = -0.15 # -0.15
        self.assembly_pose.pose.position.z = 0.601#0.601
        

        # self.assembly_pose.pose.position.x = 1.1
        # self.assembly_pose.pose.position.y = -0.10 # -0.15
        # self.assembly_pose.pose.position.z = 0.65 #0.601
        # q = tf.transformations.quaternion_from_euler(radians(20), radians(3), 0)
        # self.assembly_pose.pose.orientation.x = q[0]
        # self.assembly_pose.pose.orientation.y = q[1]
        # self.assembly_pose.pose.orientation.z = q[2]
        # self.assembly_pose.pose.orientation.w = q[3]

        self.assembly2 = "assembly"
        self.assembly2_pose = geometry_msgs.msg.PoseStamped()
        self.assembly2_pose.header.frame_id="base"
        q = tf.transformations.quaternion_from_euler(radians(85), radians(3), 0)
        self.assembly2_pose.pose.orientation.x = q[0]
        self.assembly2_pose.pose.orientation.y = q[1]
        self.assembly2_pose.pose.orientation.z = q[2]
        self.assembly2_pose.pose.orientation.w = q[3]

        self.assembly2_pose.pose.position.x = 1.1
        self.assembly2_pose.pose.position.y = 0.15
        self.assembly2_pose.pose.position.z = 0.625

        self.list = {self.assembly : self.assembly_pose}
        # self.list = {self.assembly2 : self.assembly2_pose}