# ur5_moveit_simple

Description: A demo using moveit_simple with the ur5

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

Developed by Mike Lautman at [PickNik Consulting](http://picknik.ai/)

TODO(mlautman): fix Travis badge:
[![Build Status](https://travis-ci.com/PickNikRobotics/ur5_moveit_simple.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/ur5_moveit_simple)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-kinetic-ur5_moveit_simple

### Build from Source

These instructions assume you are running on Ubuntu 16.04:

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin/
        mkdir -p $CATKIN_WS
        cd $CATKIN_WS

1. Download the required repositories and install any dependencies:

        git clone git@github.com:PickNikRobotics/ur5_moveit_simple.git
        wstool init src
        wstool merge -t src ur5_moveit_simple/ur5_moveit_simple.rosinstall
        wstool update -t src
        rosdep install --from-paths src --ignore-src --rosdistro kinetic

1. Configure and build the workspace:

        catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Developers: Quick update code repositories

To make sure you have the latest repos:

    cd $CATKIN_WS/src/ur5_moveit_simple
    git checkout master
    git pull origin master
    cd ..
    wstool merge ur5_moveit_simple/ur5_moveit_simple.rosinstall
    wstool update
    rosdep install --from-paths . --ignore-src --rosdistro kinetic

## Run

Run ur5_demo
```
roslaunch ur5_moveit_simple ur5_demo.launch
```

### Run with Debuging

Run ur5_demo with GDB
```
roslaunch ur5_moveit_simple ur5_demo.launch debug:=true
```

Run ur5_demo with Callgrind
```
roslaunch ur5_moveit_simple ur5_demo.launch callgrind:=true
```

Run ur5_demo with Valgrind
```
roslaunch ur5_moveit_simple ur5_demo.launch callgrind:=true
```

## Run Inside Docker

### Prerequisite

You must have a private rsa key `~/.ssh/id_rsa` that is not password protected and is attached to your Github/Bitbucket/Gerrit accounts.
You must also have a working installation of `docker`.

1. Navigate to `$CATKIN_WS/src/ur5_moveit_simple/.docker`. You should see the `Dockerfile` recipe in the directory.

1. Build the docker image

    cd $CATKIN_WS/src/ur5_moveit_simple/.docker
    cp ~/.ssh/id_rsa id_rsa && docker build -t ur5_moveit_simple:kinetic-source .; rm id_rsa

1. Run the docker image

    * Without the gui

            docker run -it --rm ur5_moveit_simple:kinetic-source /bin/bash

    * With the gui (tested with Ubuntu native and a Ubuntu VM)

            . ./gui-docker -it --rm ur5_moveit_simple:kinetic-source /bin/bash

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/kinetic/api/ur5_moveit_simple/html/anotated.html)

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro kinetic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
