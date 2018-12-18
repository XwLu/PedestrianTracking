#!/usr/bin/env python
import rospy
from sensor_msgs.msg import ChannelFloat32
class BBXPUB():
    def __init__(self):
        rospy.init_node('BBX', anonymous = True)
        rospy.on_shutdown(self.shutdown)

        self.pub_bbx = rospy.Publisher("/bbx", ChannelFloat32, queue_size=1)

        while not rospy.is_shutdown():
            self.Pub()
            print('Publishing!!!')
            rospy.sleep(0.1)

    def Pub(self):
        bbx = ChannelFloat32()
        bbx.values.append(0)
        bbx.values.append(1)
        bbx.values.append(2)
        bbx.values.append(3)
        self.pub_bbx.publish(bbx)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        BBXPUB()
    except rospy.ROSInterruptException:
        rospy.loginfo("BBX pub Finished.")


