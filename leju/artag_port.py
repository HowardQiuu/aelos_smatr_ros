import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import math
import tf


def get_nearest_marker():
    msg = rospy.wait_for_message('/chest/ar_pose_marker', AlvarMarkers)
    markers = []
    time_sec = msg.header.stamp.secs
    for marker in msg.markers:
        pos = marker.pose.pose.position
        quat = marker.pose.pose.orientation

        rpy = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rpy_arc = [0, 0, 0]
        for i in range(len(rpy)):
            rpy_arc[i] = rpy[i] / math.pi * 180
        

        markers.append([marker.id, pos.x, pos.y, rpy_arc[2], time_sec])
    return markers[0]


def get_specifies_marker():
    msg = rospy.wait_for_message('/chest/ar_pose_marker', AlvarMarkers)
    markers = dict()
    time_sec = msg.header.stamp.secs
    for marker in msg.markers:
        pos = marker.pose.pose.position
        quat = marker.pose.pose.orientation

        rpy = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        rpy_arc = [0, 0, 0]
        for i in range(len(rpy)):
            rpy_arc[i] = rpy[i] / math.pi * 180
        
        markers[marker.id] = [pos.x, pos.y, rpy_arc[2], time_sec]

    return markers


def tag_id():
    try:
        msg = get_nearest_marker()
        return msg[0]
    except Exception as e:
        return math.nan


def tag_x():
    try:
        msg = get_nearest_marker()
        return msg[1]
    except Exception as e:
        return math.nan


def tag_y():
    try:
        msg = get_nearest_marker()
        return msg[2]
    except Exception as e:
        return math.nan


def tag_yaw():
    try:
        msg = get_nearest_marker()
        return msg[3]
    except Exception as e:
        return math.nan

def get_specifies_tag(id):
    marker = get_specifies_marker()
    if id in marker:
        return marker[id][0], marker[id][1], marker[id][2]
    else:
        return None, None, None


if __name__ == "__main__":
    rospy.init_node("artag_port")
    print(get_specifies_tag(2))

