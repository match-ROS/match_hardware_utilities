#!/usr/bin/env python3

import rospy

class Callbacks():

    def __init__(self,ps4_driver):
        self.ps4_driver=ps4_driver

    def dummy(self):
        pass

    def increaseActiveRobotID(self):
        self.ps4_driver.active_robot = (self.ps4_driver.active_robot + 1) % len(self.ps4_driver.robotnames)
        rospy.loginfo("Increasing active robot ID to {}".format(self.ps4_driver.active_robot))


    def decreaseActiveRobotID(self):
        self.ps4_driver.active_robot = (self.ps4_driver.active_robot - 1) % len(self.ps4_driver.robotnames)
        rospy.loginfo("Decreasing active robot ID to {}".format(self.ps4_driver.active_robot))


    def increaseRot(self):
        print("Increasing rot")
        self.ps4_driver.speed_rotation=self.ps4_driver.speed_rotation * (1+self.ps4_driver.rot_incr)
        
      
    def increaseTrans(self):
        print("Increasing trans")
        self.ps4_driver.speed_translation=self.ps4_driver.speed_translation * (1+self.ps4_driver.trans_incr)
    
    def decreaseRot(self):
        print("Decreasing rot")
        self.ps4_driver.speed_rotation=self.ps4_driver.speed_rotation* (1- self.ps4_driver.rot_incr) 
        if self.ps4_driver.speed_rotation<0.0:
            self.ps4_driver.speed_rotation=0.0
      
    def decreaseTrans(self):
        print("Decreasing trans")
        self.ps4_driver.speed_translation=self.ps4_driver.speed_translation * (1-self.ps4_driver.trans_incr)
        if  self.ps4_driver.speed_translation<0.0:
            self.ps4_driver.speed_translation=0.0
    
    def changeRobot(self):
        print("Changing robot")
        # self.ps4_driver.active_robot = (self.ps4_driver.active_robot + 1) % len(self.ps4_driver.robotnames)