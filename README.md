Auburn ATTRACT
==============

Unmanned aerial vehicles (UAVs) are increasing in popularity and usability in many civil and military applications.  For a safe deployment of a fleet of UAVs operating over a limited airspace, collision avoidance is needed.  To solve this issue, Auburn University launched a project to fly autonomously, safely, and efficiently six to twelve UAVs.  This project, dubbed ATTRACT, aims to design and implement an aerial and terrestrial test bed for research and teaching in aerospace engineering, computer science, and mathematics.  The au_uav_pkg contains everything involved in the Auburn ATTRACT project.

Key Components
--------------

1. Coordinator - Monitors and coordinates all of the UAVs.

2. Collision Avoidance - Monitors all of the UAVs and decides if a collision avoidance maneuver is necessary.

3. Simulator - Mimics real UAVs by simulating their main components.

4. XBeeIO - Integrates real UAVs through a X-Bee telemetry module.

5. au_uav_gui - QT modules used to visualize the UAVs.

6. Launch files - Inherited from ROS; files used to specify which parts of the system to launch.

7. Course files - A basic system for creating a mission (for a specific UAV).


Installing ROS
--------------

Before you can clone and use the au_uav_pkg, ROS must be setup on your system.  ROS is required to run on unix-based operating systems.  A decision must be made prior to installing ROS: Installing and maintaining a personal unix operating system or using a Virtual Machine with a linux operating system.  For ease of installation, I recommend using a VM.  You can download the VM at http://nootrix.com/downloads (The Groovy Galapagos Version).  In order to install the VM, you will also need to download VirtualBox provided at http://www.virtualbox.org/wiki/Downloads.  If you choose to dual-boot operating systems or have a working version of a unix-based operating system, then simply follow the ROS installation instructions at http://www.ros.org/wiki/ROS/Installation (Make sure to follow the Groovy/catkin instructions).  

After ROS is installed, you can now fork the au_uav_pkg repository. 
 
1. Fork the "au_uav_pkg" repository by clicking the "Fork" button on the github website.  Note:  A more detailed description of forking can be found at help.github.com/articles/fork-a-repo.

2. Open a Terminal and clone this repository into your Home directory by running the following code: 
```
git clone https://github.com/dhj0001/au_uav_pkg.git
```

3. If you have setup your catkin workspace, there should be a CMakeLists.txt file in your catkin_ws/src folder.  Move this file into the au_uav_pkg/src folder.  You can do this through the Terminal by typing:
```
mv /catkin_ws/src/CMakeLists.txt /au_uav_pkg/src
```

4. Next, delete all of the files and folders in your catkin workspace by executing the following commands:
```
rm -rf catkin_ws/src
rm -rf catkin_ws/devel
rm -rf catkin_ws/build
```

5. Now, move the files and folders in au_uav_pkg (Arduino, Documentation, ExtraTools...etc) into the catkin_ws directory.  You can do this through the Terminal by executing the following commands:
```
mv /au_uav_pkg/Arduino /catkin_ws
mv /au_uav_pkg/ExtraTools /catkin_ws
mv /au_uav_pkg/Documentation /catkin_ws
mv /au_uav_pkg/src /catkin_ws
mv /au_uav_pkg/README.md /catkin_ws
mv /au_uav_pkg/.git /catkin_ws
mv /au_uav_pkg/.gitignore /catkin_ws
```

You can now set up your github Fork within your catkin_ws folder.  Refer to github.com for more instructions.

Installing QT
-------------

1. Navigate to http://qt-project.org/downloads and click on “Qt libraries 4.8.4 for Linux/X11 (225 MB)”. This will start a fairly large download that might take some time. If prompted on where to save it, just pick a folder with a path that you can easily remember. (Note: Please ensure that the correct version is downloaded. Portions of the file saving code are version-specific.)
    
2. Once the file has been downloaded, it must be extracted. This can be done by double clicking on the file and using the archive manager, or by using the terminal. If using the terminal, first change the directory to the path where you saved the file and then run `gunzip qt-everywhere-opensource-src-4.8.4.tar.gz` followed by `tar xvf qt-everywhere-opensource-src-4.8.4.tar`
    
3. Now open terminal (if it is not already open) and change the directory to the extracted folder labeled “qt-everywhere-opensource-src-4.8.4” and then run `sudo ./configure`. When prompted type ‘o’ for open source edition and “yes” to accept the license. This may take a little while to run (~30 mins).
    
4. After that is finished, run `sudo make`. This step will take several hours (up to 10 if using a virtual machine).
    
5. Once make is finished, run `sudo make install`. This will also take several hours, but less than required for “make”

6. We now need to install qt-ros, an intermediate between QT and ROS. This can be done through the terminal with `sudo apt-get install ros-groovy-qt-ros`. Type “y” at the appropriate prompt. Note: you may have to run `sudo apt-get update` first if you receive an error.

Installing QT Creator
---------------------
    
1. In order to develop the GUI, we need to install QT Creator. This allows for the user interface files to be edited and provides a very good IDE for the QT system. Once again we need to visit http://qt-project.org/downloads.  From there, we will click on “QT Creator 2.7.0 for Linux/X11 32-bit (60 MB)”.  If it isn't available on qt-project.org, you may have to search google.com for the correct version. (Note: Please ensure that the correct version is downloaded. Portions of the file saving code are version-specific.)
    
2. Once the file has downloaded, open terminal and navigate to the directory where it is saved. After this, run “sudo chmod +x qt-creator-linux-x86-opensource-2.7.0.bin”
    
3. Next, run “./qt-creator-linux-x86-opensource-2.7.0.bin”. This will open an installer. Follow the on screen steps to complete the installation (Next, Next, Agree, Install, Finish). On the last screen, be sure to uncheck the box that labeled start QT Creator. We will need to run it from the terminal instead. If you forget, simply close the instance of QT Creator that appears.

Installing ROS Dependencies
---------------------------

In order to utilize the wind simulation, GeographicLib and libnoise need to be downloaded and installed.  Lets start with GeographicLib.  

1. Retreive GeographicLib from sourceforge.net/projects/geographiclib/files/distrib (Note: This has only been tested with GeographicLib-1.30).  If prompted on where to save it, just pick a folder with a path that you can easily remember.

2. Open a Terminal and navigate to this folder.  Then type the following commands:
```
mkdir build
cd build
cmake ..
cmake .
make
make test
make install
```
This should install GeographicLib into your usr/local directory.  Navigate to this directory and ensure the lib and include directories contain GeographicLib files.  Remember this directory, because we will navigate back to it again.


Next, we will install libnoise.

1. Retreive libnoise from github.com/eXpl0it3r/libnoise.  Once again, just pick a folder to save that has a path that you can easily remember.

2. Open a Terminal and navigate to this folder.  Then type the following commands:
```
mkdir build
cd build
cmake ..
cmake .
make
make install
```

Finally, we must now copy some .so files into the catkin_ws folder.

1. First, we will use catkin_make to auto generate the devel/lib folder. Do this by entering the following commands:
```
cd catkin_ws
catkin_make
```

2. Then, enter the following commands to copy the .so files into the devel/lib folder:
```
cd ../../../
sudo cp usr/local/lib/libGeographic.so home/USERNAME/catkin_ws/devel/lib
sudo cp usr/local/lib/libGeographic.so.9 home/USERNAME/catkin_ws/devel/lib
```

ROS Build
---------

To build the au_uav_pkg navigate to your catkin_ws.  Once you are in the catkin_ws enter the following command:
```
catkin_make
```

ROS Launch
----------

To run the au_uav_pkg navigate to your catkin_ws.  Once your are in the catkin_ws, and you have already built the au_uav_pkg, launch the system by entering the following command:
```
roslaunch au_uav_ros guiDriven.launch
```
