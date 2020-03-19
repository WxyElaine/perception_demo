#! /usr/bin/env python


import rospy
import robot_api
from perception_demo_msgs.msg import TargetPose


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def move(msg, base):
	if msg.x < 0.5:  # turn left
		base.move(0, 0.2)
	elif msg.x > 0.5:  # turn right
		base.move(0, -0.2)
	if msg.depth < 10:  # move forward
		base.move(0.5, 0)
	elif msg.depth > 10:  # move backward
		base.move(-0.5, 0)


def main():
	rospy.init_node('perception_demo')
	wait_for_time()

	base = robot_api.Base()
	target_pose_sub = rospy.Subscriber('/target_pose', TargetPose, callback=move, callback_args=base)
	
	rospy.sleep(0.5)
  
	rospy.spin()

if __name__ == "__main__":
	main()
