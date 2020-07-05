# ar_commander
GNC code for analytical robotics

## Scripts description

`controller.py`: Controls robot in the
form of a desired waypoint
`/cmd\_waypoint` or trajectory
`/cmd\_trajectory` on the
`/controller\_cmds` topic.

`navigator.py`: Acts as a container for
calling a desired trajectory finding algorithm and publishes the
trajectory to `/cmd\_trajectory`.

`astar.py`: Given start, end points and
a map finds a trajectory if one exists and returns it to
`navigator.py`.

`sim\_interface.py`: serves to transfer data
to and from controller with the simulation environment, gazebo.
Conversts sim model state messages
`/gazebo/model\_states`. to the
`/pose` topic in Pose2D format. Receives
controller commands `/controller\_cmds`
and converts to sim inputs
`robot\_0/joint\$\i\\_position\_controller/command`.

`motor\_interface.py`: setups and initializes motor interface. Receives controller commands, `/controller\_cmds` and commands motors as desired. Requires operating pins of motors.

## Running the simulation and controller

1. start gazebo: `roslaunch gazebo\_interface gazebo\_sim.launch\\`
2. start controller: `python controller.py`
3. setup sim interface: `python sim\_interface.py`

Command a waypoint: * From command line:
`rostopic pub -r 10 /cmd\_waypoint geometry\_msgs/Pose2D '\x: 0.0, y: 6.0, theta: 1.0\`

Command a trajectory: * the start, end points and map can be defined in
`navigator.py` *
`python navigator.py`

## Setup Jetson Nano

## SSH into Nano
1. Connect to local wifi network on nano, configure wifi conenction to automcatiocally connect for all users in netwrok settings.
2. Check you can ping nano address from laptop : ping ---nano-ip-address---
3. SSH into nano from laptop: ssh username@---nano-ip-address---

### Configuring pins
Configure pins as desired for use, follow instructions at: [nividia_pin_docs](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html)
