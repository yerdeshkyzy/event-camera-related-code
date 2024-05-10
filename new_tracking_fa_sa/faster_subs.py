#!/usr/bin/env python3

from lib2to3.pgen2.token import NEWLINE
from logging import shutdown
import re
import rospy
import copy
import os
import time
import csv
import pandas as pd
import numpy as np
from std_msgs.msg import Int32
from tracking.msg import forces_3d
from geometry_msgs.msg import WrenchStamped
from datetime import datetime
import sys
from tracking.msg import FrankaState
from std_msgs.msg import Float32
from tracking.msg import wittenstein
from tracking.msg import centr_arr_addr

# force data variables
###############

fa_data = [0.000] * 3

# other functions' variables 
###############
elapsed = 0
counter = 0
start = time.time()
dateTimeObj = datetime.now()
filename = str(dateTimeObj.strftime("%Y-%m-%d-%H-%M-%S"))
data_row = pd.DataFrame()
data_all = pd.DataFrame()


#####################################################


def listener8_callback(data):
    global fa_data
    fa_data[0] = data.data
  

# subscribe to topic weiss  


def listener8():
    rospy.init_node('listener8', anonymous=True)
    rospy.Subscriber("/fa_pub", Int32, listener8_callback)

#####################################################
def rec():       
    global elapsed, data_all,counter, start, dateTimeObj, filename, fa_data

    counter = counter + 1

    if (counter == 1):
        dateTimeObj = datetime.now()
        filename = str(dateTimeObj.strftime("%Y-%m-%d-%H-%M-%S"))
        start = time.time()

    time_k = 10 # duration of data collection          
    
    

    if(counter > 0):

        if (elapsed <= time_k):  # some time period
            data_row = pd.DataFrame(fa_data)
            data_all = pd.concat([data_all, data_row], axis = 0)

            elapsed = time.time() - start

        if (elapsed >= time_k):
            print("stopped recording...")   
            print("saving data") 
            data_all = pd.DataFrame(data_all)

            data_2_save = copy.deepcopy(data_all)
            data_2_save.to_csv(filename + ".csv")

            data_now = pd.DataFrame()
            data_all = pd.DataFrame()
            counter = 0
            elapsed = 0
            sys.exit()




if __name__ == '__main__':
    



    listener8()
    rate = rospy.Rate(2000) #12k hz

    try:
        while not rospy.is_shutdown():
            rec()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
    
    
