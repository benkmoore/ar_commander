# This message contains the output from the controller node

# motor commands (per wheel)
std_msgs/Float64MultiArray omega_arr
std_msgs/Float64MultiArray phi_arr

# robot commands (desired velocity and omega)
std_msgs/Float64MultiArray robot_v
std_msgs/Float64 robot_omega
