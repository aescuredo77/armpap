# Armpap
Armpa: Robotic arm 4DOF with ROS Noetic

This repository hosts the source code for the ROS < ARMPApP > package.

 ![image info]()

This is part of the practices that I am doing in the course Mastering Robot orerating System ROS in the ROBOCADEMY

## Description ##

The robot type is a open-hardware Arm with 4 degrees of freedom

- Arduino mega 2560.
- Ramps 1.4 de reprap.
- Driver A4988
- wifi with ESP8266.
- Motor Nema 17 24mm x 3.
- Planetary reduction gearbox 37:1
- Gripper Motor i2c.
- RGB Smart NeoPixel x 3.


![image info](./pictures/roseco2.jpg)


### Contents ###

  - armpap_description : xacro files and meshes
  
  - armpap_control : Crontrol file, program to connect with the real robot and test with keyboard
  
  - armpap_gazebo : Simulation 
  
  - armpap_rqt_py : Plugin to control simulation and real robot
  
  - armpap_msg: specific ros messages for this type of robot.
  
  - pictures



## Key concepts covered ##
- the goal of this exercise is to get the xacro and urdf design of the robot, and to be able to simulate it in Gazebo. Also develop pluguins and drivers

- Modify the 3d arm model and add the 37: 1 reductions also printed.

  - [Reduction Source](https://www.thingiverse.com/thing:2071318)
  - [Arm Source](https://www.thingiverse.com/thing:480446)
    
  - Design a gripper
  
  ![image info](./pictures/armpap.png)


- Adapt the 3d model to simulate it. The real robot only has 4 motors, the base, right link, left link and the gripper. When creating the xacro we obtain 7 joints.
  
  - motor_base_joint (real motor nema 17)
  - link_right01_joint (real motor nema 17)
  - link_left01_joint (real motor nema 17)
  - link_left02_joint --> link_right01_joint - link_left01_joint
  - gripper_base_joint --> -link_right01_joint + 0.1
  - gripper_left_joint
  - gripper_right_joint --> Both gripper motor i2c
  
  
  ![image info](./pictures/armpapvsxacro.png)

  

- Create a Plugin in Python with QT
  
  - Use  Qt Designer to create ui.
  - Know how to use Button Widget.
  - Know how to use HorizontalSlider Widget.
  - Know how to use Checkbox Widget.
  - Know how to use label Widget.
  
  

   ![image info](./pictures/plugins.png)




## Usage ## 


To use the `armpap` package clone this repository into the `src` folder of your catkin workspace.

Then build the workspace with `catkin_make`.

Note: In my case the plugins is only compiled and appears when it is in the catkin_ws folder. If I add it to another work_space it doesn't appear.




   roslaunch armpap_description armpap_rviz.launch 

   roslaunch armpap_gazebo armpap_gazebo.launch   

   roslaunch armpap_control armpap_control.launch 


 
## Electronics : Servo motor with I2C## 

 ![](./Electronics/pictures/detail_servo_00.png)
 
 **ServoMotor model for Ecology20 and detail of the interior circuit** 
 
The servomotor consists of a controller board with a PIC16f1503 and a LB1938FA driver, which drive a Pololu &quot;Micro-metal-gearmotors&quot; type reduction motor. The reduction of the motors is based on the need for example:

- 100: 1, 130 rpm
- 298:1, 45 rpm

The microcontroller communicates through I2C, which allows us to connect 127 nodes in series. There will not be so many in our applications, we have asked for LP motors, Low power because they have a low consumption that allows us to connect them in series.

The I2c address will depend on the application that we are going to make, it should not be a problem to change it depending on the needs, but we are going to give a few premises. The I2C addresses are defined by 7 bits, the least significant bit tells us if it is writing or reading, so we will give a couple of addresses, the first is the one recorded in the pic, and the other is the one we will use in the Arduino / teensy to communicate. Always in Hexadecinal.

- Servomotor right 0x20 --> 0x10
- Servomotor left 0x22 --> 0x11

### Driver

El driver el chip [LB1938FA](https://www.dropbox.com/s/l5har1ai8nknbxs/LB1938FA.pdf?dl=0), es un driver configurado en puente-H que nos permite controlar el motor en velocidad y dirección con solo dos inputs. Analizando el driver que llevan los servomotores de DFRobot, el L9110S busque alternativas pues el susodicho driver esta un poco descatalogado o solo se vende en páginas de dudosa fiabilidad.

 ![](./Electronics/pictures/detail_driver_00.png)

 **LB1938FA is the cousin of the L9110S carried by Dfrobotics servos.** 

#### The inputs IN1 and IN2 of the driver are the RC5 and RC3 outputs of the PIC16f1503 respectively that can operate as PWM1 and PWM2.

####

### Software

 ![](./Electronics/pictures/detail_servo_01.png)
 
 **Detail of the motor control board** 

The same board can serve different purposes, for example, in the robotic arm we need positioning and hold, while the Eco\_robot we are interested in speed. For this reason and everything that the program is going to be very similar we will have to make different versions. Or group them all in one, although I am afraid that the memory of the PI16f1503 (2Kwords) is not going to give that much.

The encoder gives 12 points per turn, which must be multiplied by the reduction to know how many pulses per turn they give.

![](./Electronics/pictures/boards.jpg)

If you need more powerful motors or those that require a greater workload, there is a version of these servos with an independent power supply from the motors.

## Arduino library ##

In the Electronics folder is the MotorA library for arduino/teensy