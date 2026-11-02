# px4-ros2-drone-simulation

- B-Tech project 2021 - supervisor professor Dr. Gowtham R 

# Team member
- Bhaskar Hemanth
- Gururaj Metri 
- Harsha C
- Harshith K

* An Iris drone running PX4 firmware.
* The PX4-FastRTPS bridge to communicate with a ROS2 program that controls
the Iris drone.
* The Iris drone is simulated using Gazebo.

The versions of software used are:

* Ubuntu 18.04LTS.
* Gazebo 11.
* ROS2 Eloquent.
* The PX4 repos Firware, px4_msgs, px4_ros_com, sitl_gazebo.

## __IMPORTANT NOTES__

__Running the Gazebo server and client on a PC requires a graphics card
with at least 4GB of RAM.__  I tried an old 1GB card which worked but was
painfully slow.  I then used a laptop with an NVidia M2000 4GB card and it
worked well.

## Install and run

Docker install and run instructions can be found in `docker/README.md`.

Native install and run instructions can be found in `scripts/README.md`.

## References

Most of the inspiration for this work was taken from
<https://github.com/osrf/drone_demo> that explained how to simulate a drone in
Gazebo.  There are also countless uses of the following sites:

* <https://dev.px4.io/v1.10/en/>
* <https://discuss.px4.io/>

## Contributors

I would like to thank my teammates and my professor Dr. Gowtham R. Under his guides and we are able build this project. 
