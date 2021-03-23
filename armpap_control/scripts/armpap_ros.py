#!/usr/bin/env python3

import rospy
#from robots_control.srv import *
from robots_msg.msg import armpap_msg
from robots_msg.msg import cmd_robot_msg
#from sensor_msgs.msg import JointState
#from sensor_msgs.msg import Joy
import socket
import select
import time

pub = rospy.Publisher('pub_armpap_msg', armpap_msg, queue_size=1)
#joint_before=JointState()

class robot_armpap_msg:
    def __init__(self):
        # init ros
        rospy.init_node('armpap_node_00', anonymous=False)
        rospy.Subscriber('cmd_armpap_msg', cmd_robot_msg, self.send_cmd_to_esp8266)
        self.get_string_from_esp8266()
        
    
    def send_cmd_to_esp8266(self, data):
        message=bytearray([data.id,data.instruction,data.op1,data.op2,data.op3,data.op4,data.op5,data.id])
        sock.sendto(message, (UDP_IP, UDP_PORT))
        #time.sleep(0.05)
    

    
    def get_string_from_esp8266(self):
        global UDP_IP,UDP_PORT,id_robot
        r = rospy.Rate(10)
        pub = rospy.Publisher('pub_armpap_msg', armpap_msg, queue_size=1)
        r.sleep()
        msg = armpap_msg()
        while not rospy.is_shutdown():
            sock.setblocking(0)
            sock.connect((UDP_IP,UDP_PORT))
            ready=select.select([sock],[],[],0.05)
            if ready[0]:		
                data, addr = sock.recvfrom(1024)
                rospy.loginfo(data)
                data = data.decode('ASCII')
                dat = data.split(',') 
                rospy.loginfo( dat)           
                if dat[0] == 'ARM':
                    msg.type = dat[0]
                    msg.ip = dat[1]
                    msg.port = int(dat[2])
                    msg.id = int(dat[3])
                    msg.inst_before = int(dat[4])
                    msg.base_speed=int(dat[5])
                    msg.base_target=int(dat[6])
                    msg.base_current=int(dat[7])
                    msg.left_speed=int(dat[8])
                    msg.left_target=int(dat[9])
                    msg.left_current=int(dat[10])
                    msg.right_speed=int(dat[11])
                    msg.right_target=int(dat[12])
                    msg.right_current=int(dat[13])
                    msg.color=int(dat[14])
                    msg.gripper=int(dat[15])
                    msg.status=dat[16]
                    rospy.loginfo(msg.ip)
                    pub.publish(msg)
                    
                    #r.sleep()
            #else:
            #    message=bytearray([id_robot,21,0,0,0,0,0,id_robot]) # to open the channell
            #    sock.sendto(message, (UDP_IP, UDP_PORT))


    
   
            
        
            
            
        
    
    

if __name__ == '__main__':
    UDP_IP = "192.168.0.112"   #esp8266 ip and port
    UDP_PORT = 8888
    id_robot=254
    #joint_before = JointState()
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    message=bytearray([id_robot,21,0,0,0,0,0,id_robot])
    sock.sendto(message, (UDP_IP, UDP_PORT)) 

    try:
        robot_armpap_msg()
    except rospy.ROSInterruptException: 
        pass