#! /usr/bin/env python


import rospy
import fetch_api
from perception_demo_msgs.msg import TargetPose


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def move(msg, base):
	print(msg.x, msg.depth)
	if msg.x < 0.5:  # turn left
		base.turn(0.4)
	elif msg.x > 0.5:  # turn right
		base.turn(-0.4)
	if msg.depth >= 1000:  # move forward
		base.go_forward(.5, speed = 10)
	elif msg.depth < 1000:  # move backward
		base.go_forward(-.5, speed = 10)


def main():
	rospy.init_node('perception_demo')
	wait_for_time()

	base = fetch_api.Base()
	target_pose_sub = rospy.Subscriber('/target_pose', TargetPose, callback=move, callback_args=base)
	
	rospy.sleep(0.5)
  
	rospy.spin()

if __name__ == "__main__":
	main()
