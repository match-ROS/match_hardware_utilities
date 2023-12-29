#!/usr/bin/env python3
from PlaystationHandler import PlayStationHandler
from geometry_msgs.msg import Twist,TwistStamped
from ds4ros.srv import IsConnected, SetColor, SetColorRequest, Rumble
from mirMsgs.msg import SafetyStatus
from mir_srvs.srv import LightCommand
from std_msgs.msg import ColorRGBA
import rospy
from callbacks import Callbacks

# Controller needs to be connected, paired and trusted with bluetoothctl
# udev rule to bild Controller to /dev/input/js2
# robot_upstart launch file as systemd process
# rosservice call /light_srv "{command: 'rainbow', color1: 'ffffff', color2: 'ffffff', leds: 'all', token: '', timeout: 0.0, priority: 10000}"

class PlayStationDiffDrive(PlayStationHandler):
    
    is_Connected = False
    
    # 0 = Stopped, 1 = Wait for Release, 2 = Go
    status = 0
    prev_status = 0
    
    def __init__(self,message_type):        
        
        PlayStationHandler.__init__(self)
        self.load_config() #load config from param server
        callbacks = Callbacks(self)
        self.callbackList=[   callbacks.decreaseTrans,
                                callbacks.increaseRot,
                                callbacks.increaseTrans,
                                callbacks.decreaseRot,
                                callbacks.decreaseActiveRobotID,
                                callbacks.increaseActiveRobotID,
                                callbacks.dummy,
                                callbacks.changeRobot
                                ]

        rospy.loginfo("TODO: Implement Service vertical_movement!")
        self.v_x = float()
        self.v_y = float()
        self.rotation = float()
        self.initialized = False
        self.lower_position_reached = False
        self.publishFunction=None   
        self.publisher_stack = []       #list of publishers    
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.publishFunction=self.publishTwist
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.publishFunction=self.publishTwistStamped
        
        self.robot_publisher = rospy.Publisher("/cmd_vel",message_type,queue_size=10, latch=True)
        self.mir_listener = rospy.Subscriber('/safety_status', SafetyStatus, self.callback_mir, queue_size=10)

    # Stop driving if protective stop received and set it to "Wait for release" if status isn't driving
    def callback_mir(self, msg):
        if msg.in_protective_stop == True:
            self.status = 0
        elif self.status != 2:
            self.status = 1

    # Changes color and rumble
    def set_Color(self, red, green, blue):
        # Call Color Change service of ds4drv node
        try:
            set_color_service = rospy.ServiceProxy('/ds4drv_wrapper/set_color', SetColor)
            color_request = SetColorRequest()
            color = ColorRGBA()
            color.a = 1.0 
            color.r, color.g, color.b = red, green, blue
            color_request.color = color
            set_color_service(color_request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        
        # Rumble if color change    
        rospy.wait_for_service('/ds4drv_wrapper/rumble')
        try:
            rumble_service = rospy.ServiceProxy('/ds4drv_wrapper/rumble', Rumble)
            rumble_service(small=255, big=255, duration=0.2)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        while not rospy.is_shutdown():
            
            # Wait until controller is connected
            if not self.is_Connected:
                rospy.wait_for_service('/ds4drv_wrapper/is_connected')
                try:
                    service_client = rospy.ServiceProxy('/ds4drv_wrapper/is_connected', IsConnected)
                    response = service_client()
                    self.is_Connected = response.is_connected
                    if self.is_Connected == True:
                        self.set_Color(255, 0, 0)
                    rospy.loginfo("Verbunden: " + str(self.is_Connected))
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
                    self.is_Connected = False
                if self.is_Connected == False:
                    rospy.logerr("Kein Controller verbunden!")
                else:
                    rospy.loginfo("Controller verbunden!")
            
            # Changes to the status
            if self.prev_status != self.status:
                if self.status == 0:
                    self.set_Color(255, 0, 0)
                    self.prev_status = self.status
                    continue
                if self.status == 1:
                    self.set_Color(255, 200, 0)
                    self.prev_status = self.status
                    continue
                if self.status == 2:
                    self.set_Color(0, 255, 0)
                    self.prev_status = self.status
                    continue   
                
            # Wait for Release
            if self.status == 1 and len(self.callbackList) != 0:
                if self._axes[3] == -1 and self._axes[4] == -1:
                    self.status = 2
                    continue
            
            # Drive
            if self.status == 2 and len(self.callbackList) != 0:
                if len(self.callbackList) != 0:
                    self.v_x = (1-self._axes[4]) * self.speed_translation - (1-self._axes[3]) * self.speed_translation
                    self.rotation = (self._axes[2])*self.speed_rotation# + (self._axes[3])*self.speed_rotation
                    self.publishFunction()
            
            if self._buttons[1] == 1:
                if self.speed_translation >= 1:
                    rospy.loginfo("Max translation speed reached!")
                else:
                    self.speed_translation = self.speed_translation + self.speed_translation * 0.1
                    rospy.loginfo("Set translation speed to: %s", self.speed_translation)
                
            if self._buttons[0] == 1:
                if self.speed_translation <= 0.05:
                    rospy.loginfo("Min translation speed reached!")
                else:
                    self.speed_translation = self.speed_translation - self.speed_translation * 0.1
                    rospy.loginfo("Set translation speed to: %s", self.speed_translation)
                    
            if self._buttons[2] == 1:
                if self.speed_rotation >= 0.5:
                    rospy.loginfo("Max rotation speed reached!")
                else:
                    self.speed_rotation = self.speed_rotation + self.speed_rotation * 0.1
                    rospy.loginfo("Set rotation speed to: %s", self.speed_rotation)
                
            if self._buttons[3] == 1:
                if self.speed_rotation <= 0.05:
                    rospy.loginfo("Min rotation speed reached!")
                else:
                    self.speed_rotation = self.speed_rotation - self.speed_rotation * 0.1
                    rospy.loginfo("Set rotation speed to: %s", self.speed_rotation)
                    
            if self._buttons[4] == 1:
                rospy.wait_for_service('/light_srv')
                try:
                    light_service = rospy.ServiceProxy('/light_srv', LightCommand)
                    response = light_service(command='solid', color1='ffff00', color2='965000', leds='all', token='', timeout=0.0, priority=1001)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
                    
            if self._buttons[5] == 1:
                rospy.wait_for_service('/light_srv')
                try:
                    light_service = rospy.ServiceProxy('/light_srv', LightCommand)
                    response = light_service(command='rainbow', color1='ffffff', color2='ffffff', leds='all', token='', timeout=0.0, priority=1000)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

                

            self.rate.sleep()  

    def load_config(self):
        self.speed_translation = rospy.get_param("~translation",0.1)
        self.speed_rotation =  rospy.get_param("~rotation",0.2)
        self.trans_incr=rospy.get_param("~trans_incr",0.1)
        self.rot_incr=rospy.get_param("~rot_incr",0.1)
        self.robotname = rospy.get_param("~robot","")
        self.cmd_vel_topic_prefix = rospy.get_param("~cmd_vel_topic_prefix","")
        self.rate=rospy.Rate(rospy.get_param("~rate",10))

    def publishTwist(self):
        msg=Twist()
        rospy.loginfo("%s", msg)
        msg.linear.x=self.v_x
        msg.linear.y = self.v_y
        msg.angular.z=self.rotation
        
        self.robot_publisher.publish(msg)

    def publishTwistStamped(self):
        msg=TwistStamped()
        msg.header.stamp=rospy.Time()
        msg.header.frame_id = ''
        msg.twist.linear.x=self.v_x
        msg.twist.linear.y=self.v_y
        msg.twist.angular.z=self.rotation
        
        self.robot_publisher.publish(msg)

if __name__=="__main__":
    rospy.sleep(60)
    rospy.init_node("ps4_diffdrive_controller", anonymous=False)
    twist_stamped = True
    print(twist_stamped)
    if twist_stamped == True:
       ps4=PlayStationDiffDrive(TwistStamped)
       print("stamped")
    else:
        ps4=PlayStationDiffDrive(Twist)
        print("not stamped")
    ps4.run()
    rospy.spin()
