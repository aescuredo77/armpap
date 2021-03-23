string type 		# type of robot  ”ARMPAP”
string ip 		# address of the robot ESP8866
uint32 port 		# Port for comunication
uint8 id 		# identyti of robot a uint8_t
uint8 inst_before 	# the last instruction runner 
uint16 base_speed 	# speed base
int16 base_target 	# base target position
int16 base_current 	# base current position
uint16 left_speed 	# speed left
int16 left_target 	# left target position
int16 left_current 	# left current position
uint16 right_speed 	# speed right
int16 right_target 	# right target position
int16 right_current 	# right current position
uint8 color  		# color of the light 0--17
uint8 gripper 		# status of the gripper 0 open, 1 close
string status 		# "Completed" or "Error tipo 3"
