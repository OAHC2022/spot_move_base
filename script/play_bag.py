#!/usr/bin/env python
import rospy 

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Laserpoint_cloud
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from scipy.spatial.transform import Rotation as R

import rosbag 
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt
import tf2_ros

from nav_msgs.msg import Path

rospy.init_node('talker', anonymous=True)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
point_cloud_pub = rospy.Publisher("/point_cloud", PointCloud2, queue_size=10)
tf_pub = tf2_ros.TransformBroadcaster()

pub_goal = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)

traj_pub = rospy.Publisher(
            '/expected_path', Path, queue_size=1)

bag = rosbag.Bag("../bags/135.bag")

all_odom = []
first_location = None
for topic, msg, t in bag.read_messages():
    if topic == "/odom":
        odom = msg
        if first_location == None:
            first_location = [odom.pose.pose.position.x, odom.pose.pose.position.y]      
        
        odom.pose.pose.position.x -= first_location[0]
        odom.pose.pose.position.y -= first_location[1]
        odom.pose.pose.position.z = 0
        all_odom.append([odom, t])

count = 0
point_cloud_count = 0

def sleep(wait_time):
    old_time = rospy.Time.now()
    while True:
        if (rospy.Time.now() - old_time).to_sec() > wait_time:
            break

for topic, msg, t in bag.read_messages():
    if topic == "/odom":

        odom = msg
        now_time = rospy.Time.now()
        odom.header.stamp = now_time
        odom.pose.pose.position.x -= first_location[0]
        odom.pose.pose.position.y -= first_location[1]
        odom.pose.pose.position.z = 0
        # print(odom)

        tf = TransformStamped()
        tf.header.stamp = now_time
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'body'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = odom.pose.pose.orientation.x
        tf.transform.rotation.y = odom.pose.pose.orientation.y
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        tf_pub.sendTransform(tf)

        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'velodyne'
        tf.transform.translation.x = 0
        tf.transform.translation.y = 0
        tf.transform.translation.z = 0.86
        tf.transform.rotation.x = 0
        tf.transform.rotation.y = 0
        tf.transform.rotation.z = 0
        tf.transform.rotation.w = 1
        tf_pub.sendTransform(tf)
        odom_pub.publish(odom)
        count += 1
        print('ok')
        
    if topic == "/velodyne_points":
        point_cloud = msg
        point_cloud.header.stamp = rospy.Time.now()
        point_cloud_pub.publish(msg)
        point_cloud_count += 1
        if point_cloud_count % 20 != 0:
            continue

        # synchronize
        t1 = abs(all_odom[count-1][1] - t).to_sec()
        t2 = abs(all_odom[count][1] - t).to_sec()

        if t1 < t2 and t1 <= 0.05:
            index = count - 1
 
        elif t1 > t2 and t2 <= 0.05:
            index = count 

        # find 10m away goal
        odom_or = all_odom[index][0]
        path = Path()
        path.header.frame_id = "odom"
        for id in range(index + 1, len(all_odom)):
            odom = all_odom[id][0]
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.pose = odom.pose.pose
            path.poses.append(pose_stamped)

            if (odom.pose.pose.position.x - odom_or.pose.pose.position.x)**2 + (odom.pose.pose.position.y - odom_or.pose.pose.position.y)**2 >= 25:
                break
        odom = all_odom[id][0]
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position = odom.pose.pose.position
        orie = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        orie = R.from_quat(orie).as_euler('xyz', degrees=True)[2]
        orie = R.from_euler('xyz', [0, 0, orie], degrees=True).as_quat()

        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = orie
        pub_goal.publish(goal)
        traj_pub.publish(path)
        flag = True
        # input()
        
        # print(goal)
        print("sleep 6s~")
        sleep(6)
        print("unsleep")
        
    
    sleep(0.05)
    

print(count)