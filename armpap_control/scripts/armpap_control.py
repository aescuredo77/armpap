#!/usr/bin/env python3

import os
#import rospkg
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64
from robots_msg.msg import armpap_msg
from robots_msg.msg import cmd_robot_msg
from robots_class import arm_robot


from PyQt5 import QtWidgets, uic
import sys
import time
import math


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('armpap_control.ui', self)
        self.setWindowTitle("Control_armpap")
        rospy.init_node('armpap_operate', anonymous=False)
        #rp = rospkg.RosPack()
        self.realRobot = False
        self.arm = arm_robot("192.168.0.112", 254, 8888)
        self.aux = 0
        
        # Sequence define with some movements
        self.seq = [[0,0,0]             # reposo 0
                ,[1400,600,0]           # reposo 1
                ,[1400,1200,2000]       # reposo 2
                ,[0,1800,500]           # A  
                ,[0,1800,0]             # A'
                ,[1400,2600,100]        # B
                ,[1400,2200,0]          # B'
                ,[1400,1300,1400]       # C
                ,[1400,900,800]         # C'
                ,[1800,1900,1000]       # D
                ,[1800,1500,600]        # D'
                ,[3400,900,0]           # E
                ,[4400,900,0]]

        # Ros publisher
        self.sub = rospy.Subscriber('/pub_armpap_msg', armpap_msg, self.read_status)
        self.pub = rospy.Publisher('/talker', String, queue_size=1)
        self.real_robot = rospy.Publisher('/cmd_armpap_msg', cmd_robot_msg, queue_size=1)
        self.base_motor = rospy.Publisher('/armpap/motor_base_joint_controller/command', Float64, queue_size=10)
        self.left_motor = rospy.Publisher('/armpap/link_left01_joint_controller/command', Float64, queue_size=10)
        self.right_motor = rospy.Publisher('/armpap/link_right01_joint_controller/command', Float64, queue_size=10)
        self.aux_right_motor = rospy.Publisher('/armpap/link_left02_joint_controller/command', Float64, queue_size=10)
        self.base_gripper = rospy.Publisher('/armpap/gripper_base_joint_controller/command', Float64, queue_size=10)
        self.left_gripper = rospy.Publisher('/armpap/gripper_left_joint_controller/command', Float64, queue_size=10)
        self.right_gripper = rospy.Publisher('/armpap/gripper_right_joint_controller/command', Float64, queue_size=10)

        

        self.button1 = self.findChild(QtWidgets.QPushButton, 'button1') # Find the button
        self.button2 = self.findChild(QtWidgets.QPushButton, 'button2') # Find the button
        self.reset = self.findChild(QtWidgets.QPushButton, 'reset') # Find the button
        #self.graphicsView = self.findChild(QtWidgets.QGraphicsView, 'graphicsView') 
        self.checkBox = self.findChild(QtWidgets.QCheckBox, 'checkBox') # Find the checkbox
        self.horizontalSlider_0 = self.findChild(QtWidgets.QSlider, 'horizontalSlider_0') # Find the horizontalSlider_0
        self.horizontalSlider_1 = self.findChild(QtWidgets.QSlider, 'horizontalSlider_1') # Find the horizontalSlider_1
        self.horizontalSlider_2 = self.findChild(QtWidgets.QSlider, 'horizontalSlider_2') # Find the horizontalSlider_2
        self.label_0 = self.findChild(QtWidgets.QLabel, 'label_0') # Find the label_0
        self.label_1 = self.findChild(QtWidgets.QLabel, 'label_1') # Find the label_1
        self.label_2 = self.findChild(QtWidgets.QLabel, 'label_2') # Find the label_2
        self.label_h0 = self.findChild(QtWidgets.QLabel, 'label_h0') # Find the label_h0
        self.label_h1 = self.findChild(QtWidgets.QLabel, 'label_h1') # Find the label_h1
        self.label_h2 = self.findChild(QtWidgets.QLabel, 'label_h2') # Find the label_h2
        #self.line1 = self.findChild(QtWidgets.Line, 'line_1')
        #self.line2 = self.findChild(QtWidgets.Line, 'line_2')
        #self.line3 = self.findChild(QtWidgets.Line, 'line_3')
        #self.line4 = self.findChild(QtWidgets.Line, 'line_4')
        
        self.groupbox_1 = self.findChild(QtWidgets.QGroupBox, 'groupBox_1')
        self.connected = self.findChild(QtWidgets.QLabel, 'connected')
        self.type = self.findChild(QtWidgets.QLabel, 'type')
        self.ip = self.findChild(QtWidgets.QLabel, 'ip')
        self.port = self.findChild(QtWidgets.QLabel, 'port')
        self.id = self.findChild(QtWidgets.QLabel, 'id')
        self.instbefore = self.findChild(QtWidgets.QLabel, 'instbefore')
        self.speed = self.findChild(QtWidgets.QLabel, 'speed')
        self.basecurrent_2 = self.findChild(QtWidgets.QLabel, 'basecurrent_2')
        self.leftcurrent = self.findChild(QtWidgets.QLabel, 'leftcurrent')
        self.rightcurrent = self.findChild(QtWidgets.QLabel, 'rightcurrent')
        self.color = self.findChild(QtWidgets.QLabel, 'color')
        self.gripper = self.findChild(QtWidgets.QLabel, 'gripper')
        self.status = self.findChild(QtWidgets.QLabel, 'status')
        self.info_label01 = self.findChild(QtWidgets.QLabel, 'info_label01') # type
        self.info_label02 = self.findChild(QtWidgets.QLabel, 'info_label02') # ip
        self.info_label03 = self.findChild(QtWidgets.QLabel, 'info_label03') # port
        self.info_label04 = self.findChild(QtWidgets.QLabel, 'info_label04') # id
        self.info_label05 = self.findChild(QtWidgets.QLabel, 'info_label05') # ints before
        self.info_label06 = self.findChild(QtWidgets.QLabel, 'info_label06') # speed
        self.info_label07 = self.findChild(QtWidgets.QLabel, 'info_label07') # base current position
        self.info_label08 = self.findChild(QtWidgets.QLabel, 'info_label08') # left current position
        self.info_label09 = self.findChild(QtWidgets.QLabel, 'info_label09') # right current position
        self.info_label10 = self.findChild(QtWidgets.QLabel, 'info_label10') # color
        self.info_label11 = self.findChild(QtWidgets.QLabel, 'info_label11') # gripper
        self.info_label12 = self.findChild(QtWidgets.QLabel, 'info_label12') # status
       
        self.groupbox_2 = self.findChild(QtWidgets.QGroupBox, 'groupBox_2')
        self.playseq = self.findChild(QtWidgets.QPushButton, 'playseq')
        self.saveseq = self.findChild(QtWidgets.QPushButton, 'saveseq')
        self.saveseq_2 = self.findChild(QtWidgets.QSpinBox, 'saveseq_2')
        self.saveseq_2.setMaximum(len(self.seq))
        self.numberseq = self.findChild(QtWidgets.QSpinBox, 'numberseq')
        self.numberseq.setMaximum(len(self.seq)-1)
        self.base_play_seq = self.findChild(QtWidgets.QLabel, 'base_play_seq')
        self.left_play_seq = self.findChild(QtWidgets.QLabel, 'left_play_seq')
        self.right_play_seq = self.findChild(QtWidgets.QLabel, 'right_play_seq')
        self.base_save_seq = self.findChild(QtWidgets.QLabel, 'base_save_seq')
        self.left_save_seq = self.findChild(QtWidgets.QLabel, 'left_save_seq')
        self.right_save_seq = self.findChild(QtWidgets.QLabel, 'right_save_seq')
        self.info_label_base01 = self.findChild(QtWidgets.QLabel, 'info_label_base01')
        self.info_label_left01 = self.findChild(QtWidgets.QLabel, 'info_label_left01')
        self.info_label_right01 = self.findChild(QtWidgets.QLabel, 'info_label_rigth01')
        self.info_label_base02 = self.findChild(QtWidgets.QLabel, 'info_label_base02')
        self.info_label_left02 = self.findChild(QtWidgets.QLabel, 'info_label_left02')
        self.info_label_right02 = self.findChild(QtWidgets.QLabel, 'info_label_rigth02')

        
            

        #self.button1.clicked.connect(self.printButtonPressed) # Remember to pass the definition/method, not the return value!

        self.button1.clicked.connect(self.on_click0)
        self.button2.clicked.connect(self.on_click1)
        self.playseq.clicked.connect(self.playseq_on_click)
        self.saveseq.clicked.connect(self.saveseq_on_click)
        self.reset.clicked.connect(self.start_position)
        self.saveseq_2.valueChanged.connect(self.savesseq_change)
        self.numberseq.valueChanged.connect(self.numberseq_change)
        self.horizontalSlider_0.sliderReleased.connect(self.changeValue0)
        self.horizontalSlider_1.sliderReleased.connect(self.changeValue1)
        self.horizontalSlider_2.sliderReleased.connect(self.changeValue2)
        self.horizontalSlider_0.valueChanged.connect(self.labelchange0)
        self.horizontalSlider_1.valueChanged.connect(self.labelchange1)
        self.horizontalSlider_2.valueChanged.connect(self.labelchange2)
        self.checkBox.stateChanged.connect(self.checkBoxChangedAction)
        self.label_h0.setText(str(self.horizontalSlider_0.value()))
        self.label_h1.setText(str(self.horizontalSlider_1.value()))
        self.label_h2.setText(str(self.horizontalSlider_2.value()))

        #self.start_position()





        self.show()
    

    def read_status(self,data):
        self.connected.setText('Connected')
        self.type.setText(data.type)
        self.ip.setText(data.ip) 
        self.port.setText(str(data.port))
        self.id.setText(str(data.id))
        self.instbefore.setText(str(data.inst_before))
        self.speed.setText(str(data.base_speed))
        self.basecurrent_2.setText(str(data.base_current))
        self.leftcurrent.setText(str(data.left_current))
        self.rightcurrent.setText(str(data.right_current))
        self.color.setText(str(data.color))
        if data.gripper == 0:
            self.gripper.setText('close')
        else:
            self.gripper.setText('open')

        self.status.setText(data.status)



    
    
    def start_position(self):    # reset initial position 
        command = Float64()
        command.data = -1.5                                         # base
        self.base_motor.publish(command)
        self.horizontalSlider_0.setValue(0)
        self.label_h0.setText(str(self.horizontalSlider_0.value()))
        command.data = 0.4                                          # rigth
        self.right_motor.publish(command)
        self.horizontalSlider_2.setValue(0)
        self.label_h2.setText(str(self.horizontalSlider_2.value()))
        command.data = -0.6
        self.aux_right_motor.publish(command)
        command.data = 0.2
        self.base_gripper.publish(command)
        command.data = 1.1                                          # left
        self.left_motor.publish(command)
        self.horizontalSlider_1.setValue(0)
        self.label_h1.setText(str(self.horizontalSlider_1.value()))
        if(self.realRobot):
           self.real_robot.publish(self.arm.inst(97)[0])
        

    def on_click0(self):     # open gripper
        message = String()
        message.data="Open Gripper"
        command = Float64()
        command.data = 0.0
        self.left_gripper.publish(command)
        self.right_gripper.publish(command)
        if(self.realRobot):
            self.real_robot.publish(self.arm.inst(37)[0])
        rospy.loginfo(message)   
        self.pub.publish(message)

    def on_click1(self):    # close gripper
        message = String()
        message.data="Close Gripper"
        command1 = Float64()
        command1.data = 0.02
        command2 = Float64()
        command2.data = -0.02
        self.left_gripper.publish(command2)
        self.right_gripper.publish(command1)
        if(self.realRobot):
            self.real_robot.publish(self.arm.inst(36)[0])
        rospy.loginfo(message)   
        self.pub.publish(message)

    def changeValue0(self):  #  move base
        value = self.horizontalSlider_0.value()
        message = String()
        message.data="Move base armpap"
        command = Float64()
        command.data = value *  0.055  - 1.5 # -1,5..4,0 --> 0 .. 99
        self.base_motor.publish(command)
        if(self.realRobot):
            self.aux = value * 50 # 0..5000
            aux1 = (self.aux >> 8) & 0xff
            aux2 = self.aux & 0xff
            print(aux1)
            print(aux2)
            print(self.arm.inst(1,0,aux1,aux2))
            self.real_robot.publish(self.arm.inst(1,0,aux1,aux2)[0])
        rospy.loginfo(message)   
        self.pub.publish(message)
        

    def changeValue1(self): # move left
        value = self.horizontalSlider_1.value()
        message = String()
        message.data="Move left armpap " 
        left1 = 0.0
        left1 = 1.1 - value *  0.018
        right1 = 0.0 
        right1 = 0.4 - self.horizontalSlider_2.value() * 0.007
        command = Float64()
        command.data = left1                  #  1,1..-0,8 --> 0 .. 99
        commandl2 = Float64()
        commandl2.data = right1-left1
        aux_command = Float64()
        aux_command = -right1 + 0.1
        self.aux_right_motor.publish(commandl2)
        self.base_gripper.publish(aux_command)
        self.left_motor.publish(command)
        if(self.realRobot):
            self.aux = value * 30  # 0..3000
            print(self.aux)
            aux1 = (self.aux >> 8) & 0xff
            aux2 = self.aux & 0xff
            self.real_robot.publish(self.arm.inst(1,1,aux1,aux2)[0])
        rospy.loginfo(message)   
        self.pub.publish(message)

    def changeValue2(self): # right
        value = self.horizontalSlider_2.value()
        message = String()
        message.data="Move right armpap"
        left1 = 0.0
        left1 = 1.1 - self.horizontalSlider_1.value() *  0.018
        right1 = 0.0 
        right1 = 0.4 - value * 0.007
        command = Float64()
        command.data = right1               #  0,4..-0,2 --> 0..99
        commandl2 = Float64()
        commandl2.data = right1 - left1
        aux_command = Float64()
        aux_command = -right1 + 0.1
        self.right_motor.publish(command)
        self.aux_right_motor.publish(commandl2)
        self.base_gripper.publish(aux_command)  
        if(self.realRobot):
            self.aux = value * 20  # 0..2000
            aux1 = (self.aux >> 8) & 0xff
            aux2 = self.aux & 0xff
            self.real_robot.publish(self.arm.inst(1,2,aux1,aux2)[0])
        rospy.loginfo(message)   
        self.pub.publish(message) 

    def checkBoxChangedAction(self, state):
        message = String()
        if(state):
            self.realRobot = True
            message.data = "Working with the real Robot too"
        else:
            self.realRobot = False
            message.data = "Working with the simulation Robot"
        rospy.loginfo(message)   
        self.pub.publish(message)
    
    
    def labelchange0(self,value):
        self.label_h0.setText(str(value * 50))
    
    def labelchange1(self,value):
        self.label_h1.setText(str(value * 30))
    
    def labelchange2(self,value):
        self.label_h2.setText(str(value * 20))

    def savesseq_change(self):
        if self.saveseq_2.value() == len(self.seq):
            self.base_save_seq.setText('0')
            self.left_save_seq.setText('0')
            self.right_save_seq.setText('0')
        else:
            aux_seq = self.seq[self.saveseq_2.value()]
            self.base_save_seq.setText(str(aux_seq[0]))
            self.left_save_seq.setText(str(aux_seq[1]))
            self.right_save_seq.setText(str(aux_seq[2]))
    
    def saveseq_on_click(self):
        if len(self.seq) > self.saveseq_2.value():  #  change existing seq
            self.seq[self.saveseq_2.value()] = [self.horizontalSlider_0.value() * 50, self.horizontalSlider_1.value() * 30, self.horizontalSlider_2.value() * 20]
            self.base_save_seq.setText(str(self.horizontalSlider_0.value() * 50))
            self.left_save_seq.setText(str(self.horizontalSlider_1.value() * 30))
            self.right_save_seq.setText(str(self.horizontalSlider_2.value() * 20))
        else:
            self.seq.append([self.horizontalSlider_0.value() * 50, self.horizontalSlider_1.value() * 30, self.horizontalSlider_2.value() * 20])
            self.saveseq_2.setMaximum(len(self.seq))
            self.numberseq.setMaximum(len(self.seq)-1)
            #self.saveseq_2.setValue(len(self.seq))
            self.base_save_seq.setText(str(self.horizontalSlider_0.value() * 50))
            self.left_save_seq.setText(str(self.horizontalSlider_1.value() * 30))
            self.right_save_seq.setText(str(self.horizontalSlider_2.value() * 20))
        print(self.seq)

        
    def sim_move(self,pos_):
        aux_pos_base = Float64()
        aux_pos_left = Float64()
        aux_pos_right = Float64()
        aux_pos_aux_right = Float64()
        aux_pos_base_gripper = Float64()
        aux_pos_base.data = (pos_[0]/50) * 0.055
        aux_pos_left.data = 1.1 - (pos_[1]/30) * 0.018
        aux_pos_right.data = 0.4 - (pos_[2]/20) * 0.007
        aux_pos_aux_right.data = aux_pos_right.data - aux_pos_left.data
        aux_pos_base_gripper.data = -aux_pos_right.data + 0.1

        return aux_pos_base, aux_pos_left, aux_pos_right, aux_pos_aux_right, aux_pos_base_gripper

    def numberseq_change(self):
        aux_seq = self.seq[self.numberseq.value()]
        self.base_play_seq.setText(str(aux_seq[0]))
        self.left_play_seq.setText(str(aux_seq[1]))
        self.right_play_seq.setText(str(aux_seq[2]))

    def playseq_on_click(self):
        aux_seq = self.seq[self.numberseq.value()]
        base_, left_, right_, aux_r_, base_g_ = self.sim_move(aux_seq)
        self.base_motor.publish(base_)
        self.left_motor.publish(left_)
        self.right_motor.publish(right_)
        self.aux_right_motor.publish(aux_r_)
        self.base_gripper.publish(base_g_)
        if (self.realRobot):
            self.real_robot.publish(self.arm.inst(2,0,(aux_seq[0] >> 8) & 0xff, aux_seq[0] & 0xff)[0])
            time.sleep(0.2)
            self.real_robot.publish(self.arm.inst(2,1,(aux_seq[1] >> 8) & 0xff, aux_seq[1] & 0xff)[0])
            time.sleep(0.2)
            self.real_robot.publish(self.arm.inst(2,2,(aux_seq[2] >> 8) & 0xff, aux_seq[2] & 0xff)[0])
            time.sleep(0.2)
            self.real_robot.publish(self.arm.inst(3)[0])



        






if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()
    


    
