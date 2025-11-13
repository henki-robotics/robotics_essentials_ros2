## Installing Ubuntu 22.04.5 and all the course dependencies in UTM for Apple users

Machine specification: Apple Macbook Pro M1 Max, 32 GB RAM.

1) Install the [UTM virtualization software](https://mac.getutm.app/).

2) Download the 
[Ubuntu 22.04.5 server version for ARM architecture (aarch)](https://cdimage.ubuntu.com/releases/22.04.5/release/).
   3) Why 22.04 and not the recent one? ROS 2 Humble is officially supported in Ubuntu 22.04 LTS. There are newer 
   versions of ROS 2 as well, but we are using 
some packages that are tested to be compatible with ROS 2 Humble. 
   4) Why the server version? We are sure if there is a pre-built desktop image for Ubuntu 22.04 ARM. There is something 
   for Raspberry Pi but not sure how it would work on laptops. 

3) After installing Ubuntu server from the image you downloaded, update the system 
(`sudo apt-get update && sudo apt-get upgrade`). Then install the `ubuntu-desktop` package. It takes a while... 
After rebooting, we have a nicely working Ubuntu desktop virtualized on Mac! 

4) Seems that everything works, so we proceed to install ROS 2 Humble. Follow the 
[instructions here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

5) After successfully installing ROS 2 and verifying that it works (see the ROS 2 documentation), we install the 
`andino_gz` packages for the [simulated Andino robot](https://github.com/Ekumen-OS/andino_gz). Simply, issue the command
`sudo apt install ros-humble-andino-gz` and that will do the trick! 

6) Verify that Andino works by launching the environment in a new terminal: `ros2 launch andino_gz andino_gz.launch.py`.
It would result something like in the image below.

![andino_env_working](images/Nayttokuva_2025-10-30_kello_22.38.19.webp)

7) Now you can follow assignments in [our repository](https://github.com/henki-robotics/robotics_essentials_ros2).
Obviously you will have to repeat the [0-setup](../0-setup) section. NOTE! When using this installation you must skip 
any Docker-related instructions or steps, since you will work with ROS 2 Humble that is straight installed system-wide
on the Ubuntu running inside UTM.

## Tips and tricks:
- [Here is a good tutorial that can also be followed](https://techblog.shippio.io/how-to-run-an-ubuntu-22-04-vm-on-m1-m2-apple-silicon-9554adf4fda1). 
- If Gazebo crashes right at the beginning, you can issue the command `export LIBGL_ALWAYS_SOFTWARE=1` to enforce 
software rendering for OpenGL software. Obviously this is not ideal, as everything runs on the CPU in our case, but at 
least on the machine these instructions were tested, even this rendered decent performance. Be aware that your CPU 
will get quite HOT!
  - Alternatively, you can add the `LIBGL_ALWAYS_SOFTWARE=1` line to your `~/.bashrc` file to apply it to every new 
  terminal shell that you open. Likewise, you can add the `source /opt/ros/humble/setup.bash` command to your 
  `~/.bashrc` so that you don't have to issue that command always when opening a new shell for ROS 2 use. Please, 
  see the official ROS 2 documentation for more information.

## Acknowledgements
The instructions in this tutorial were put together by Prof. 
[Ilkka Jormanainen](https://www.linkedin.com/in/ilkka-jormanainen-5954441/).