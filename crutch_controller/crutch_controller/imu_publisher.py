"""
 ********************************************************************************
 *
 *     BBBBBBB      IIII   NNN     N   AA       TTTTTTTT   AA
 *     B      B      II    N N     N   AAA        TTTT     AAA
 *     B      BB     II    N  N    N   AAAA        TT      AAAA
 *     BBBBBBBBB     II    N   N   N   AAAAA       TT      AAAAA
 *     B        BB   II    N   N   N   AA   A      TT      AA   A
 *     B        BB   II    N    N  N   AA    A     TT      AA    A
 *     BBBBBBBBBB   IIII   N    NNNN   AA     A   TTTT     AA     A
 *
 ********************************************************************************
 * All software (C)
 ********************************************************************************
"""

#python publisher 

#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Accel #https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


# print(" *******************************************************************************")
# print(" *")
# print(" *     BBBBBBB      IIII   NNN     N   AA       TTTTTTTT   AA")
# print(" *     B      B      II    N N     N   AAA        TTTT     AAA")
# print(" *     B      BB     II    N  N    N   AAAA        TT      AAAA")
# print(" *     BBBBBBBBB     II    N   N   N   AAAAA       TT      AAAAA")
# print(" *     B        BB   II    N   N   N   AA   A      TT      AA   A")
# print(" *     B        BB   II    N    N  N   AA    A     TT      AA    A")
# print(" *     BBBBBBBBBB   IIII   N    NNNN   AA     A   TTTT     AA     A")
# print(" *")
# print(" *******************************************************************************")
# print(" * All software (C, py, C++)")
# print(" *******************************************************************************")
# time.sleep(1)

# print("")
# print("                 _           _           _    _                    ")
# print("     /\         (_)         | |         | |  | |                   ")
# print("    /  \   _ __  _ ___   ___| |__   __ _| | _| | _____  _   _ _ __ ")
# print("   / /\ \ | '_ \| / __| / __| '_ \ / _` | |/ / |/ / _ \| | | | '__|")
# print("  / ____ \| | | | \__ \ \__ \ | | | (_| |   <|   < (_) | |_| | |   ")
# print(" /_/    \_\_| |_|_|___/ |___/_| |_|\__,_|_|\_\_|\_\___/ \__,_|_|   ")
# print("")
# print("")
# time.sleep(2)


class IMU_Publisher(Node): #my class inherets the functionality of ROS2 nodes
    def __init__(self):
        super().__init__("imu_publisher") #Nodes name must be equal to the node
        self.get_logger().info("Starting") #nodes info
        
        #creating timer
        self.create_timer(1.0, self.timer_callback) #from ROS2 nodes
        self.msg_counter = 0
        ## creating a 
        self.pub_pos = self.create_publisher(Float32MultiArray, '/forward_position_controller/commands', 10) #publishing to controller
        self.cmd_pub_Quat = self.create_publisher(Quaternion, "turtle1/cmd_vel", 10) # Data type, topic name, queue size
        self.cmd_pub_Accl = self.create_publisher(Accel, "turtle1/cmdv_el", 10) # Data type, topic name, queue size


    def timer_callback(self):
        msg_quat = Quaternion()
        msg_quat.x = 1.0
        msg_quat.y = 1.0
        msg_quat.z = 1.0
        msg_quat.w = 1.0

        msg_acc = Accel()
        msg_acc.linear.x = 1.0
        msg_acc.linear.y = 3.0
        msg_acc.linear.z = 7.0
        
        self.cmd_pub_Quat.publish(msg_quat)
        self.cmd_pub_Accl.publish(msg_acc)
        
        pos_array = Float32MultiArray(data=[self.msg_counter])
        self.pub_pos(pos_array)

        self.get_logger().info("message " + str(self.msg_counter))
        self.msg_counter += 1

def main(args=None):
    rclpy.init(args=args) #init ROS2 communications 
    running_Node = IMU_Publisher() #create a node which inherets ROS2 nodes funtionality
    rclpy.spin(running_Node) #keeps the node alive until it is killed by CTRL c
    running_Node.destroy_node()
    rclpy.shutdown() #close ROS2 communication 
    pass



#if we want tot directly excetute the program 
if __name__ == '__main__':
    main()