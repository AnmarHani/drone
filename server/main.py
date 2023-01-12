from pymavlink import mavutil
from dataclasses import dataclass

@dataclass
class Drone:
    the_connection: mavutil
    connection_url: str
    waypoints = []

    def __init__(self, url: str = 'udpin:localhost:14551'):
        self.the_connection = mavutil.mavlink_connection(url)
        self.connection_url = url
        self.the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))

    def return_to_launch(self, parameters: list[int] = [0, 0, 0, 0, 0, 0, 0, 0], change_mode: bool = True) -> None:
        RETURN_TO_LAUNCH_CMD = 20

        # return to launch
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system, 
            self.the_connection.target_component,
            RETURN_TO_LAUNCH_CMD, 
            *parameters
        )
        return {
        "message": self.check()      
        }

    def arm (self, parameters: list[int] = [0,0,0,0,0,0]) -> None :
        ARM_OR_DISARM_CMD = 400 
        ARM: int = 1

        self.the_connection.mav.command_long_send(
            self.the_connection.target_system, 
            self.the_connection.target_component, 
            ARM_OR_DISARM_CMD, 
            0, 
            ARM, 
            *parameters
        )
        return {
        "message": self.check()      
        }
        
    

       
        

    def disarm (self, parameters: list[int] = [0,0,0,0,0,0]) -> None :
        ARM_OR_DISARM_CMD = 400 
        DISARM: int = 0

        self.the_connection.mav.command_long_send(
            self.the_connection.target_system, 
            self.the_connection.target_component, 
            ARM_OR_DISARM_CMD, 
            0, 
            DISARM, 
            *parameters
        )
        return {
        "message": self.check()      
        }
        

    def takeoff(self , parameters: list[int] = [0, 0, 0, 0, 0, 0, 0] , arm: bool = False, altitude: int = 10) -> None :

        TAKEOFF_CMD = 22 

        if(arm == True): self.arm()

        self.the_connection.mav.command_long_send(
            self.the_connection.target_system, 
            self.the_connection.target_component,  
            TAKEOFF_CMD, 
            *parameters, 
            altitude
        )
        return {
        "message": self.check()      
        }
        

    def local_movement(self, 
        depth_position: int, 
        horizontal_position: int, 
        vertical_position: int,
        depth_velocity: int = 0, 
        horizontal_velocity: int = 0, 
        vertical_velocity: int = 0,
        depth_acceleration: int = 0, 
        horizontal_acceleration: int = 0, 
        vertical_acceleration: int = 0,
        rotation: int = 0,
        rotation_degree: int = 0)  -> None :

        BOOT_TIME_MILLISECONDS: int = 10

        FORWARD_BACKWARD_POSITION_AXIS: int = depth_position
        RIGHT_LEFT_POSITION_AXIS: int = horizontal_position
        UP_DOWN_POSITION_AXIS: int = -1 * vertical_position

        FORWARD_BACKWARD_VELOCITY_AXIS: int = depth_velocity
        RIGHT_LEFT_VELOCITY_AXIS: int = horizontal_velocity
        UP_DOWN_VELOCITY_AXIS: int = -1 * vertical_velocity

        FORWARD_BACKWARD_ACCELERATION_AXIS: int = depth_acceleration
        RIGHT_LEFT_ACCELERATION_AXIS: int = horizontal_acceleration
        UP_DOWN_ACCELERATION_AXIS: int = -1 * vertical_acceleration

        YAW: int = rotation
        YAW_DEGREE: int = rotation_degree

        FRAME_TYPE = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
        
        IGNORE_X_POSITION: bool = False
        IGNORE_Y_POSITION: bool = False
        IGNORE_Z_POSITION: bool = False

        IGNORE_X_VELOCITY: bool = True if depth_velocity == 0 else False
        IGNORE_Y_VELOCITY: bool = True if horizontal_velocity == 0 else False
        IGNORE_Z_VELOCITY: bool = True if vertical_velocity == 0 else False

        IGNORE_X_ACCELERATION: bool = True if depth_acceleration == 0 else False
        IGNORE_Y_ACCELERATION: bool = True if horizontal_acceleration == 0 else False
        IGNORE_Z_ACCELERATION: bool = True if vertical_acceleration == 0 else False

        USE_FORCE: bool = False
        
        IGNORE_YAW: bool = True if rotation == 0 else False
        IGNORE_YAW_RATE: bool = True if rotation_degree == 0 else False
        
        MASK_RULES: str = str(
            str(int(IGNORE_YAW_RATE))+
            str(int(IGNORE_YAW))+
            str(int(USE_FORCE))+
            str(int(IGNORE_Z_ACCELERATION))+
            str(int(IGNORE_Y_ACCELERATION))+
            str(int(IGNORE_X_ACCELERATION))+
            str(int(IGNORE_Z_VELOCITY))+
            str(int(IGNORE_Y_VELOCITY))+
            str(int(IGNORE_X_VELOCITY))+
            str(int(IGNORE_Z_POSITION))+
            str(int(IGNORE_Y_POSITION))+
            str(int(IGNORE_X_POSITION))
        )
        
        

        SET_LOCAL_POSITION_CMD = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            BOOT_TIME_MILLISECONDS, 

            self.the_connection.target_system,
            self.the_connection.target_component, 

            FRAME_TYPE, 
            
            int("0b"+MASK_RULES, 2), #first, convert MASK_RULES to binary by adding 0b in the first of the string. Second, convert that string binary to an actual binary by int("string_binary_here", base_2) will result in an actual binary instead of a binary as a string

            FORWARD_BACKWARD_POSITION_AXIS, 
            RIGHT_LEFT_POSITION_AXIS,
            UP_DOWN_POSITION_AXIS, 

            FORWARD_BACKWARD_VELOCITY_AXIS,
            RIGHT_LEFT_VELOCITY_AXIS,
            UP_DOWN_VELOCITY_AXIS,

            FORWARD_BACKWARD_ACCELERATION_AXIS,
            RIGHT_LEFT_ACCELERATION_AXIS,
            UP_DOWN_ACCELERATION_AXIS,  
            
            YAW,
            YAW_DEGREE
        )
        self.the_connection.mav.send(SET_LOCAL_POSITION_CMD)
        return {
        "message": self.check()      
        }
    
    def global_movement(self, 
        latitude_position: int, 
        longtitude_position: int, 
        altitude_position: int,
        latitude_velocity: int = 0, 
        longtitude_velocity: int = 0, 
        altitude_velocity: int = 0,
        latitude_acceleration: int = 0, 
        longtitude_acceleration: int = 0, 
        altitude_acceleration: int = 0,
        rotation: int = 0,
        rotation_degree: int = 0)  -> None :

        BOOT_TIME_MILLISECONDS: int = 10

        FORWARD_BACKWARD_POSITION_AXIS: int = latitude_position
        RIGHT_LEFT_POSITION_AXIS: int = longtitude_position
        UP_DOWN_POSITION_AXIS: int = -1 * altitude_position

        FORWARD_BACKWARD_VELOCITY_AXIS: int = latitude_velocity
        RIGHT_LEFT_VELOCITY_AXIS: int = longtitude_velocity
        UP_DOWN_VELOCITY_AXIS: int = -1 * altitude_velocity

        FORWARD_BACKWARD_ACCELERATION_AXIS: int = latitude_acceleration
        RIGHT_LEFT_ACCELERATION_AXIS: int = longtitude_acceleration
        UP_DOWN_ACCELERATION_AXIS: int = -1 * altitude_acceleration

        YAW: int = rotation
        YAW_DEGREE: int = rotation_degree

        FRAME_TYPE = mavutil.mavlink.MAV_FRAME_GLOBAL
        
        IGNORE_X_POSITION: bool = False
        IGNORE_Y_POSITION: bool = False
        IGNORE_Z_POSITION: bool = False

        IGNORE_X_VELOCITY: bool = True if depth_velocity == 0 else False
        IGNORE_Y_VELOCITY: bool = True if horizontal_velocity == 0 else False
        IGNORE_Z_VELOCITY: bool = True if vertical_velocity == 0 else False

        IGNORE_X_ACCELERATION: bool = True if depth_acceleration == 0 else False
        IGNORE_Y_ACCELERATION: bool = True if horizontal_acceleration == 0 else False
        IGNORE_Z_ACCELERATION: bool = True if vertical_acceleration == 0 else False

        USE_FORCE: bool = False
        
        IGNORE_YAW: bool = True if rotation == 0 else False
        IGNORE_YAW_RATE: bool = True if rotation_degree == 0 else False
        
        MASK_RULES: str = str(
            str(int(IGNORE_YAW_RATE))+
            str(int(IGNORE_YAW))+
            str(int(USE_FORCE))+
            str(int(IGNORE_Z_ACCELERATION))+
            str(int(IGNORE_Y_ACCELERATION))+
            str(int(IGNORE_X_ACCELERATION))+
            str(int(IGNORE_Z_VELOCITY))+
            str(int(IGNORE_Y_VELOCITY))+
            str(int(IGNORE_X_VELOCITY))+
            str(int(IGNORE_Z_POSITION))+
            str(int(IGNORE_Y_POSITION))+
            str(int(IGNORE_X_POSITION))
        )

        SET_GLOBAL_POSITION_CMD = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            BOOT_TIME_MILLISECONDS, 

            self.the_connection.target_system,
            self.the_connection.target_component, 

            FRAME_TYPE, 
            
            int("0b"+MASK_RULES, 2), #first, convert MASK_RULES to binary by adding 0b in the first of the string. Second, convert that string binary to an actual binary by int("string_binary_here", base_2) will result in an actual binary instead of a binary as a string

            FORWARD_BACKWARD_POSITION_AXIS, 
            RIGHT_LEFT_POSITION_AXIS,
            UP_DOWN_POSITION_AXIS, 

            FORWARD_BACKWARD_VELOCITY_AXIS,
            RIGHT_LEFT_VELOCITY_AXIS,
            UP_DOWN_VELOCITY_AXIS,

            FORWARD_BACKWARD_ACCELERATION_AXIS,
            RIGHT_LEFT_ACCELERATION_AXIS,
            UP_DOWN_ACCELERATION_AXIS,  
            
            YAW,
            YAW_DEGREE
        )
        self.the_connection.mav.send(SET_GLOBAL_POSITION_CMD)
        return {
        "message": self.check()      
        }

    def add_new_waypoint(self, longtitude: float, latitude: float, altitude: float):
        location: dict[str, float] = {
            "latitude": latitude,
            "longtitude": longtitude,
            "altitude": altitude
        }
        self.waypoints.append(location)

        

    def delete_all_waypoints(self):
        self.waypoints.clear()




    def move_to_waypoints(self, hold_time: int = 5, accept_radius: int = 10, pass_radius: int = 5, yaw: int = 0):
        NAV_WAYPOINT_CMD: int = 16

        HOLD_TIME: int = hold_time

        ACCEPT_RADIUS_METERS: int = accept_radius
        PASS_RADIUS_METERS: int = pass_radius

        YAW: int = yaw

        for waypoint in self.waypoints:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                NAV_WAYPOINT_CMD,
                0,
                HOLD_TIME,
                ACCEPT_RADIUS_METERS,
                PASS_RADIUS_METERS,
                YAW,
                waypoint["latitude"],
                waypoint["longtitude"],
                waypoint["altitude"]
            )



    def change_mode(self , mode: str = 'GUIDED'):
        MAV_CHANGE_MODE_COMMAND = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 

        mode_id = self.the_connection.mode_mapping()[mode]
        self.the_connection.mav.set_mode_send(
            self.the_connection.target_system,
            MAV_CHANGE_MODE_COMMAND,
            mode_id
        )
        return {
        "message": self.check()      
        }



    def sync_with_drone(self):
        GPS_INFO = self.the_connection.recv_match(type="GPS_RAW_INT", blocking=True)
        LOCAL_POSITION_INFO = self.the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        SYSTEM_STATUS_INFO = self.the_connection.recv_match(type="SYS_STATUS", blocking=True)
        HEARTBEAT_INFO = self.the_connection.recv_match(type="HEARTBEAT", blocking=True)
        
        self.latitude = GPS_INFO.lat / 10 ** 7
        self.longtitude= GPS_INFO.lon / 10 ** 7
        self.altitude= LOCAL_POSITION_INFO.z * -1
        self.battery_remaining = SYSTEM_STATUS_INFO.battery_remaining
        if(HEARTBEAT_INFO.custom_mode == 3):
            self.mode = "AUTO"
        if(HEARTBEAT_INFO.custom_mode == 4):
            self.mode = "GUIDED"
        if(HEARTBEAT_INFO.custom_mode == 6):
            self.mode = "RTL"

            

    def check(self) : 
        check = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True )
        if (check.result == 0)  : return "Command is valid "  
        if (check.result == 1)  : return "Command is valid, but cannot be executed at this time"
        if (check.result == 2)  : return "Command is invalid"
        if (check.result == 3)  : return "Command is not supported"
        if (check.result == 4)  : return "Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem"
        if (check.result == 5)  : return "Command is valid and is being executed."   
        if (check.result == 6)  : return "Command has been cancelled"  
        

Drone1 = Drone()
Drone1.arm()
Drone1.takeoff(altitude=15)




############ SOME EXPLAINATIONS ############
# Message protocol: https://mavlink.io/en/messages/common.html
# description: get data from the drone

# function = the_connection.recv_match("BATTERY_STATUS")

# parameters =  (type: must be a Message type inside a string), (blocking: means stop and wait until the drone give u the response then continue code)

# name = MAV_CMD_REQUEST_MESSAGE in the documentation

# return = returns an object which refers to the Message type, where u can get inside and get the enum of that Message type u can see on documentation and that enum may return an integer so u can check what this integer means in the docs
# ------------------------------------------------------------------------------------------------
# Mission protocol: https://mavlink.io/en/services/mission.html#mavlink_commands
# description: upload a mission item to the drone

# function = the_connection.mission_item_send

# parameters =  (target_system), (target_component), (command: must be a MAV_CMD type as an integer value), (4 parameters for the MAV_CMD), (latitude), (longitude), (altitude), (frame: enum for MAV_FRAME), (seq: mission number), (mission_type: int enum for MAV_MISSION_TYPE but 0 is good) , (current: make 0 its false), (autocontinue: when mission 1 finish continue mission 2? true/false)

# name = MISSION_ITEM_INT in the documentation

# return = nothing
# ------------------------------------------------------------------------------------------------
# Command protocol: https://mavlink.io/en/services/command.html
# description: to give drone a command to run

# function = the_connection.command_long_send

# parameters =  (target_system), (target_component), (command: must be a MAV_CMD type as an integer value) (other parameters for specified MAV_CMD type)

# name = COMMAND_LONG in the documentation

# return = nothing

############ HOW TO SET THE DRONE MODE HERE ############
# DO_SET_MODE_CMD = 176
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE

############ MISSION COMMANDS TO MAKE THE DRONE AUTOMATICALLY DO THE MISSION ############
# GOOD VIDEO
# https://www.youtube.com/watch?v=pAAN055XCxA&t=616s&ab_channel=AscendEngineering

# # i dont know but I think u have to put it
# drone1.the_connection.mav.mission_count_send(
#     drone1.the_connection.target_system, 
#     drone1.the_connection.target_component, 
#     0, 
#     0
# )

# # check if the drone can accept a MISSION request now or no
# drone1.the_connection.recv_match(type="MISSION_REQUEST", blocking=True)

# # upload a mission item to the drone so it will be saved
# NAV_WAYPOINT_CMD: int = 16

# drone1.the_connection.mav.mission_item_send(
#     target_system=drone1.the_connection.target_system, 
#     target_component=drone1.the_connection.target_component, 
#     seq=0, # sequence (means the mission count, 0 is the first mission)
#     current=0, # just make it zero
#     autocontinue=1, # when finish mission 0 go to mission 1 automaticaly .....
#     mission_type=0, # just make it zero 
#     command= NAV_WAYPOINT_CMD,
#     frame= mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # global frame
#     x=-35.3604843, # latitude
#     y=149.1722724, # longitude
#     z=14, # altitude
#     param1=0,param2=10,param3=5,param4=0 #other parameters
# )

# # check if the mission item is added and accepted
# drone1.the_connection.recv_match(type="MISSION_ACK", blocking=True)

# # now start the mission and go through every mission item
# MISSION_START_CMD: int = 300

# drone1.the_connection.mav.command_long_send(
#     drone1.the_connection.target_system, 
#     drone1.the_connection.target_component, 
#     MISSION_START_CMD, # to start the mission
#     0,0,0,0,0,0,0,0)

# # check the command status to know if its accepted or no 
# drone_info =  drone1.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
# print(drone_info.result)