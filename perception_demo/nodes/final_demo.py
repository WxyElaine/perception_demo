#! /usr/bin/env python


import rospy
import robot_api
from perception_demo_msgs.msg import TargetPose
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tft
import numpy as np

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class CubeCarrier():

    def __init__(self):
        # search for the person only after the cube is picked up
        self._cube_picked = False

        # Fetch controls
        self._base = robot_api.Base()
        self._arm = robot_api.Arm()
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._fetch_gripper = robot_api.Gripper()
        self._move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self._move_base_client.wait_for_server()
        # transformation
        self._tf_listener = tf.TransformListener()
        self._viz_pub = rospy.Publisher('debug_marker', Marker, queue_size=5)


        self._cube_centroid_sub = rospy.Subscriber('/cube_centroid', PoseStamped, callback=self.pickup_cube)
        self._target_pose_sub = rospy.Subscriber('/target_pose', TargetPose, callback=self.goto_target)

        rospy.sleep(0.5)

        # raise torso
        self._torso.set_height(0.2)
        # look down
        self._head.pan_tilt(0, 0.3)


    def pickup_cube(self, msg):
        if not self._cube_picked:
            # print(msg)
            # move base to the table
            # self._base.go_forward(1, speed=0.2)
            # self._base.stop()
            

            # goal = MoveBaseGoal()
            # goal.target_pose.header.frame_id = "map"
            # goal.target_pose.header.stamp = rospy.Time.now()
            # goal.target_pose.pose.position.x = 1.8
            # goal.target_pose.pose.position.y =  0.15
            # goal.target_pose.pose.position.z =  0.83
            # goal.target_pose.pose.orientation.w = 1

            # self._move_base_client.send_goal(goal)
            # wait = self._move_base_client.wait_for_result()
            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            # else:
            #     # move_base_client.get_result()
            #     print "Ready to pick up the cube!"

            # pick up the cube
            (position, quaternion) = self._tf_lookup()
            if (position, quaternion) != (None, None):
                # get the transformation, record it
                record_pose = Pose()
                record_pose.position.x = position[0]
                record_pose.position.y = position[1]
                record_pose.position.z = position[2]
                record_pose.orientation.x = quaternion[0]
                record_pose.orientation.y = quaternion[1]
                record_pose.orientation.z = quaternion[2]
                record_pose.orientation.w = quaternion[3]

                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = "base_link"

                marker = Marker(type=Marker.ARROW,
                    id=1,
                    pose=record_pose,
                    scale=Vector3(1, 0.01, 0.01),
                    header=ps.header,
                    color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
                self._viz_pub.publish(marker)

                marker = Marker(type=Marker.ARROW,
                    id=2,
                    pose=msg.pose,
                    scale=Vector3(1, 0.01, 0.01),
                    header=ps.header,
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
                self._viz_pub.publish(marker)

                t_mat = self._pose_to_transform(record_pose)
                w_mat = self._pose_to_transform(msg.pose)
                new_trans = np.dot(np.linalg.inv(t_mat), w_mat)
                pose = self._transform_to_pose(new_trans)

                ps.pose = pose
                ps.pose.orientation.w = 1.57

                marker = Marker(type=Marker.ARROW,
                    id=0,
                    pose=ps.pose,
                    scale=Vector3(1, 0.01, 0.01),
                    header=ps.header,
                    color=ColorRGBA(1.0, 0.75, 0.3, 0.8))
                self._viz_pub.publish(marker)
                print "marker"
                error = self._arm.move_to_pose(ps)
                if error is not None:
                    self._arm.cancel_all_goals()
                    rospy.logerr("Fail to move: {}".format(error))
                else:
                    # succeed
                    print "Cube picked up!"
                    self._cube_picked = True


    def goto_target(self, msg):
        if self._cube_picked:
            print(msg.x, msg.depth)
            if msg.x < 0.5:  # turn left
                self._base.turn(0.4)
            elif msg.x > 0.5:  # turn right
                self._base.turn(-0.4)
            if msg.depth >= 1000:  # move forward
                self._base.go_forward(.5, speed = 10)
            elif msg.depth < 1000:  # move backward
                self._base.go_forward(-.5, speed = 10)

    def _tf_lookup(self):
        """ 
            Lookups the transformation between "map" and "base_link" (retry up to TRANSFROM_LOOKUP_RETRY times),
            and returns the result
        """
        (position, quaternion) = (None, None)
        count = 0
        while True:
            if (position, quaternion) != (None, None):  # lookup succeeds
                return (position, quaternion)
            elif count >= 20:  # exceeds maximum retry times
                rospy.logerr("Fail to lookup transfrom information between 'map' and 'base_link'")
                return (None, None)
            else:  # try to lookup transform information
                try:
                    (position, quaternion) = self._tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    count += 1
                    continue
    

    def _transform_to_pose(self, matrix):
        """ Matrix to pose """
        pose = Pose()
        trans_vector = tft.translation_from_matrix(matrix)
        pose.position = Point(trans_vector[0], trans_vector[1], trans_vector[2])
        quartern = tft.quaternion_from_matrix(matrix)
        pose.orientation = Quaternion(quartern[0], quartern[1], quartern[2], quartern[3])
        return pose


    def _pose_to_transform(self, pose):
        """ Pose to matrix """
        q = pose.orientation
        matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z
        return matrix


def main():
    rospy.init_node('final_demo')
    wait_for_time()
    CubeCarrier()
    rospy.spin()

if __name__ == "__main__":
	main()
