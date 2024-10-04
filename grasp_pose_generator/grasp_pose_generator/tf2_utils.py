# Helper Functions for interfacing with TF2 in ROS 2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Quaternion, Pose
import tf2_ros
import tf2_geometry_msgs


# Lazy create on use (convert_pose) to avoid errors.
tfBuffer = None
listener = None


def _init_tf(node):
    # Create buffer and listener in ROS 2 (requires node as context)
    global tfBuffer, listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer, node)


def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def list_to_quaternion(l):
    q = Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]
    return q


def convert_pose(node, pose, from_frame, to_frame):
    """
    Convert a pose or transform between frames using tf2 in ROS 2.
    - pose: A Pose that defines the robot's position and orientation in the reference frame.
    - from_frame: A string defining the original reference frame of the robot.
    - to_frame: A string defining the desired reference frame to convert to.
    """
    global tfBuffer, listener

    if tfBuffer is None or listener is None:
        _init_tf(node)

    try:
        # Use the current time in ROS 2
        trans = tfBuffer.lookup_transform(to_frame, from_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        node.get_logger().error(f'FAILED TO GET TRANSFORM FROM {from_frame} to {to_frame}: {e}')
        return None

    spose = PoseStamped()
    spose.pose = pose
    spose.header.stamp = node.get_clock().now().to_msg()
    spose.header.frame_id = from_frame

    p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

    return p2.pose


def current_robot_pose(node, reference_frame, base_frame):
    """
    Get the current pose of the robot in the given reference frame.
    """
    p = Pose()
    p.orientation.w = 1.0

    return convert_pose(node, p, base_frame, reference_frame)


def publish_stamped_transform(node, stamped_transform, seconds=1):
    """
    Publish a stamped transform for debugging purposes.
    """
    br = tf2_ros.TransformBroadcaster(node)

    stamped_transform.header.stamp = node.get_clock().now().to_msg()
    br.sendTransform(stamped_transform)

    # Publish transform for the set time duration
    iterations = int(seconds / 0.05)
    for _ in range(iterations):
        stamped_transform.header.stamp = node.get_clock().now().to_msg()
        br.sendTransform(stamped_transform)
        rclpy.sleep(0.05)


def publish_transform(node, transform, reference_frame, name, seconds=1):
    """
    Publish a Transform for debugging purposes.
    """
    st = TransformStamped()
    st.transform = transform
    st.header.frame_id = reference_frame
    st.child_frame_id = name

    publish_stamped_transform(node, st, seconds)


def publish_pose_as_transform(node, pose, reference_frame, name, seconds=1):
    """
    Publish a Pose as a transform for visualization in RViz.
    """
    t = TransformStamped()
    t.header.frame_id = reference_frame
    t.child_frame_id = name
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation

    publish_stamped_transform(node, t, seconds)


def publish_tf_quaternion_as_transform(node, translation, quaternion, reference_frame, name, seconds=1):
    """
    Publish a transform defined by a translation and quaternion.
    """
    qm = Transform()
    qm.translation.x = translation[0]
    qm.translation.y = translation[1]
    qm.translation.z = translation[2]
    qm.rotation.x = quaternion[0]
    qm.rotation.y = quaternion[1]
    qm.rotation.z = quaternion[2]
    qm.rotation.w = quaternion[3]

    publish_transform(node, qm, reference_frame, name, seconds)


def align_pose_orientation_to_frame(node, from_pose, from_reference_frame, to_reference_frame):
    """
    Align the orientation of from_pose to match the orientation of to_reference_frame.
    """
    p = Pose()
    p.orientation.w = 1.0

    pose_orientation = convert_pose(node, p, to_reference_frame, from_reference_frame)

    from_pose.orientation = pose_orientation.orientation

    return from_pose
