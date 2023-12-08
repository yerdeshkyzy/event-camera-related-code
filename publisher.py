#!/usr/bin/env python
# license removed for brevity
from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import String, Bool
from threading import Thread

boolPub=False

def talker():
    pub = rospy.Publisher('collection_in_progress', Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    thread = Thread(target=changebool)
    thread.start()
    global boolPub
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rospy.loginfo(boolPub)
        pub.publish(boolPub)
        rate.sleep()

def changebool():
    while(True):
        num=input("Write your bool")
        global boolPub 
        boolPub = True if num=="1" else False



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
