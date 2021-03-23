#!/usr/bin/env python3
import rospy

from robots_msg.msg import cmd_robot_msg


import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving :
    base  left   right  conveyer     gripper    
    q +   w +    e +    r  to 2500   y  close 
    a -   s -    d -    f  to 0      h  open
    
    p fail
    o rearm to init 
    i rearm pause
    x close
       
n/m : increase/decrease sequence
+/- : increase/decrease speed, default +

"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 2.5
turn = 3.5
joint1=0.0
joint2=0.0

def print_msg(msg):
    print ("                                                                               ", end='\r')
    print ("   "+ msg, end='\r')

def control_armpap(ints, commad, sp):
    msg=cmd_robot_msg()
    msg.ip="192.168.0.112"
    msg.port=8888
    msg.id=254
    msg.instruction=ints
    msg.op1=commad
    msg.op2=sp #default change to move less or more max 255
    msg.op3=0
    msg.op4=0
    msg.op5=0
    #pub.publish(msg)
    return msg

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('armpap_teleop')
    pub = rospy.Publisher('cmd_armpap_msg', cmd_robot_msg, queue_size=1)
    
    #r=rospy.Rate(5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    max_speed = 5.0
    max_turn = 5.0
    max_joint = 0.32
    push=0
    cmd_msg=cmd_robot_msg()
    cmd_before=cmd_robot_msg()
    base=0
    left=0
    right=0
    seq=0
    speed=0
    try:
        print (msg)
        while(1):
            key = getKey()

            if key == 'q': #base +
                cmd_msg=control_armpap(10,113,200)
                base += 200
                if base > 5000:
                    base = 5000
                count = 0
                print_msg("increment base " + str(base) )
            elif key == 'a': #base -
                cmd_msg=control_armpap(10,97,200)
                base -= 200
                if base < 0:
                    base = 0
                count = 0
                print_msg("decrement base " + str(base) )
            elif key == 'w': #left +
                cmd_msg=control_armpap(10,119,200)
                left += 200
                if left > 3000:
                    left = 3000
                count = 0
                print_msg("increment  " + str(left) )
            elif key == 's': #left -
                cmd_msg=control_armpap(10,115,200)
                left -= 200
                if left < 0:
                    left = 0
                count = 0
                print_msg("decrement  " + str(left) )
            elif key == 'e': #right +
                cmd_msg=control_armpap(10,101,200)
                right += 200
                if right > 2000:
                    right = 2000
                count = 0
                print_msg("increment  " + str(right) )
            elif key == 'd': #right -
                cmd_msg=control_armpap(10,100,200)
                right -= 200
                if right < 0:
                    right = 0
                count = 0
                print_msg("decrement  " + str(right) )
            elif key == 'r': #conveyer belt to 25000
                cmd_msg=control_armpap(10,114,speed)
                count = 0
                print_msg("conveyer belt to 25000")
            elif key == 'f': #forward right
                cmd_msg=control_armpap(10,102,speed)
                print_msg("conveyer belt to 0")
            elif key == 'y': #backward left
                cmd_msg=control_armpap(10,121,speed)
                count = 0
                print_msg("Open Gripper   ¿?")
            elif key == 'h': #backward right
                cmd_msg=control_armpap(10,104,speed)
                count = 0
                print_msg("Close Gripper  ¿?")
            elif key == 'p': #stop
                cmd_msg=control_armpap(10,112,speed)
                count = 0
                print_msg("STOP EMERGENCY ON HOLD") 
            elif key == 'o': # restart
                cmd_msg=control_armpap(10,111,speed)
                count = 0
                print_msg("RESTART") 
            elif key == 'i': #restart from position
                cmd_msg=control_armpap(10,105,speed)
                count = 0
                print_msg("CONTINUED")
            elif key == 'm': #increment sequence to play
                seq +=1
                if seq > 17: 
                    seq = 17
                cmd_msg=control_armpap(9,seq,speed)
                count = 0
                print_msg("play sequence " + str(seq)) 
            elif key == 'n': #restart from position
                seq -= 1
                if seq < 0: 
                    seq = 0
                cmd_msg=control_armpap(9,seq,speed)
                count = 0
                print_msg("play sequence " + str(seq))
            elif key == '-':
                speed = 1
                count = 0
                print_msg("Speed is 1200")
            elif key == '+':
                speed = 0
                count = 0
                print_msg("Speed is 600")
            elif key == 'x':
                break 
            else:
                count = count + 1
                if count > 2:
                    cmd_msg=control_armpap(21,0,speed) # ros message
                    count = 0
                    print_msg("{" + str(base) + ", " + str(left) + ", " + str(right) + "}") 

            if cmd_msg != cmd_before :
                pub.publish(cmd_msg)
                cmd_before = cmd_msg
            

    except:
        print ("algun error ha pasado, culpa de Oscar")

    finally:
        cmd_msg=control_armpap(21,0) # ros message
        pub.publish(cmd_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)