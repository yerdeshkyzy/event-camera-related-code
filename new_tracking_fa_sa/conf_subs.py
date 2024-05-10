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
ebts_force = [0.000] * 3
weiss_force = [0.000] * 3
desired_force = [0.000] * 3
delta_z = [0.000]*3
pixelxyz = [0.000]*3
witt_force = [0.000] * 3
centr_arr1 =  [0.000] * 3
centr_arr2 =  [0.000] * 3
centr_arr3 =  [0.000] * 3
centr_arr4 =  [0.000] * 3
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
def listener2_callback(data):
    global ebts_force
    ebts_force[0] = data.force_x
    ebts_force[1] = data.force_y
    ebts_force[2] = data.force_z

def listener1_callback(data):
    global weiss_force
    weiss_force[0] = data.wrench.force.x
    weiss_force[1] = data.wrench.force.y
    weiss_force[2] = data.wrench.force.z

def deltaz_callback(data):
    global delta_z
    
    delta_z[0] = data.O_T_EE[14] # z
    delta_z[1] = data.O_T_EE[12] # x
    delta_z[2] = data.O_T_EE[13] # y
    #print(delta_z)

def pixel_callback(data):
    global pixelxyz

    pixelxyz[0] = data.force_x
    pixelxyz[1] = data.force_y
    pixelxyz[2] = data.force_z
    #print(alldist)

def listener5_callback(data):
    global desired_force
    desired_force[0] = data.wrench.force.x
    desired_force[1] = data.wrench.force.y
    desired_force[2] = data.wrench.force.z

def listener6_callback(data):
    global witt_force 
    witt_force[0] = data.fx
    witt_force[1] = data.fy
    witt_force[2] = data.fz

def listener7_callback(data):
    global centr_arr1, centr_arr2, centr_arr3, centr_arr4
    centr_arr1[0] = data.marker0_x
    centr_arr1[1] = data.marker0_y
    centr_arr1[2] = data.marker1_x

    centr_arr2[0] = data.marker1_y
    centr_arr2[1] = data.marker2_x
    centr_arr2[2] = data.marker2_y

    centr_arr3[0] = data.marker3_x
    centr_arr3[1] = data.marker3_y
    centr_arr3[2] = data.marker4_x

    centr_arr4[0] = data.marker4_y


def listener8_callback(data):
    global fa_data
    fa_data[0] = data.data
  

# subscribe to topic weiss  
def listener1():
    rospy.init_node('listener1', anonymous=True)
    rospy.Subscriber("/wrench", WrenchStamped, listener1_callback)

# subscribe to topic ebts
def listener2():
    rospy.Subscriber("forces_3d_pub", forces_3d, listener2_callback)
# z axis of the
def listener3():
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, deltaz_callback)    
def listener4():
    rospy.Subscriber('/pixel_pub', forces_3d, pixel_callback)    

def listener5():
    # rospy.init_node('listener1', anonymous=True)
    rospy.Subscriber("/desired_force", WrenchStamped, listener5_callback)

def listener6():
    rospy.Subscriber("/wittenstein_topic", wittenstein, listener6_callback)

def listener7():
    rospy.Subscriber("/centre_pub", centr_arr_addr, listener7_callback)

def listener8():
    rospy.Subscriber("/fa_pub", Int32, listener8_callback)

#####################################################
def rec():       
    global elapsed, data_all, data_row, counter, start, dateTimeObj, filename, weiss_force, ebts_force, delta_z, pixelxyz, desired_force, witt_force, centr_arr1, centr_arr2, centr_arr3, centr_arr4, fa_data

    counter = counter + 1

    if (counter == 1):
        dateTimeObj = datetime.now()
        filename = str(dateTimeObj.strftime("%Y-%m-%d-%H-%M-%S"))
        start = time.time()

    time_k = 60 # duration of data collection          
    
    

    if(counter > 0):

        if (elapsed <= time_k):  # some time period
            data_row = pd.concat( [pd.DataFrame(witt_force), pd.DataFrame(ebts_force), pd.DataFrame(delta_z),  pd.DataFrame(pixelxyz), pd.DataFrame(desired_force), pd.DataFrame(centr_arr1),  pd.DataFrame(centr_arr2),  pd.DataFrame(centr_arr3),  pd.DataFrame(centr_arr4),  pd.DataFrame(fa_data)], axis = 0)
            data_all = pd.concat([data_all, data_row.transpose()], axis = 0)

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
    


    listener1()
    listener2()
    listener3()
    listener4()
    listener5()
    listener6()
    listener7()
    listener8()
    rate = rospy.Rate(2000) #10hz

    try:
        while not rospy.is_shutdown():
            rec()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
    
    
