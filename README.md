# ros_hello
ROS HelloWorld

## Table of Contents
- [Simple Publisher and Subscriber](#simple-publisher-and-subscriber)
- [Simple Service and Client](#simple-service-and-client)
- [Building and running gtest](#building-and-running-gtest)
- [Launching multiple ROS nodes](#launching-multiple-ros-nodes)
- [Parameter Server](#parameter-server)
- [Dynamically reconfigure](#dynamically-reconfigure)
- [Simple Action Server and Client](#simple-action-server-and-client)

---

## Requirements
- ROS Melodic Morenia
- [Googletest](https://github.com/google/googletest)

## Building
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/akhud78/ros_hello.git
$ cd ~/catkin_ws && catkin_make
```

## Usage
### Simple Publisher and Subscriber
#### C++
- [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- Run roscore and talker node
```
$ cd ~/catkin_ws
$ roscore &
$ rosrun ros_hello hello_talker
[ INFO] [1589802474.900453477]: hello world 0
[ INFO] [1589802475.400594265]: hello world 1
...
```
- Run listener node in another terminal
```
$ cd ~/catkin_ws
$ rosrun ros_hello hello_listener
[ INFO] [1589803049.554629825]: I heard: [hello world 31]
[ INFO] [1589803050.054440248]: I heard: [hello world 32]
...
^C
```
- Stop roscore in first terminal
```
[ INFO] [1589803053.553447324]: hello world 39
[ INFO] [1589803054.053424323]: hello world 40
^C
```
#### Python
- [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- Run talker.py
```
$ rosrun ros_hello talker.py
[INFO] [1589808962.491005]: hello python world 1589808962.49
[INFO] [1589808963.492489]: hello python world 1589808963.49
```
- Run listener python node in another terminal
```
$ rosrun ros_hello listener.py
...
^C
```
- Stop server in first terminal and shutdown roscore
```
$ fg
roscore
^C[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

### Simple Service and Client
- [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
```
$ roscore &
$ rosrun ros_hello add_two_ints_server
[ INFO] [1590073190.727817864]: Ready to add two ints.
```
- Run client node in another terminal
```
$ rosservice list
/add_two_ints
...
$ rosservice args /add_two_ints
a b
$ rosservice call /add_two_ints 1 2
sum: 3
```
- Stop server in first terminal and shutdown roscore

### Building and running gtest
- [Building and running tests](http://wiki.ros.org/gtest)
- Run test and shutdown roscore
```
$ roscore &
$ rosrun ros_hello hello_test_utils
...
[==========] 1 test from 1 test case ran. (1 ms total)
[  PASSED  ] 1 test.
```
### Launching multiple ROS nodes
- [roslaunch](http://wiki.ros.org/roslaunch)
- A roslaunch will **automatically start roscore** if it detects that it is not already running!
- [Roslaunch tips for large projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects)
- [Roslaunch, управление запуском](http://docs.voltbro.ru/starting-ros/administrirovanie-ros/roslaunch.html)
```
$ roslaunch ros_hello demo.launch --screen
...
[ INFO] [1592465254.021331150]: hello world 1
[ INFO] [1592465254.022126986]: I heard: [hello world 1]
[INFO] [1592465254.266100]: hello python world 1592465254.27
[INFO] [1592465254.269992]: /ns2/listenerI heard hello python world 1592465254.27
...
$ rostopic list
/ns1/chatter
/ns2/chatter
```
- [substitution args](http://wiki.ros.org/roslaunch/XML)
- Run without Python nodes
```
$ roslaunch ros_hello demo.launch --screen py:=false
...
[ INFO] [1592465402.682849741]: hello world 1
[ INFO] [1592465402.683594511]: I heard: [hello world 1]
[ INFO] [1592465403.682807483]: hello world 2
[ INFO] [1592465403.683026892]: I heard: [hello world 2]
...
$ rostopic list
/ns1/chatter
```

### Parameter Server
- [Parameter Server](http://wiki.ros.org/roscpp/Overview/Parameter%20Server)
```
$ roslaunch ros_hello demo.launch --screen py:=0 name:=Andy
...
[ INFO] [1590072398.495229414]: hello Andy 1
[ INFO] [1590072398.495879860]: I heard: [hello Andy 1]
```

### Dynamically reconfigure
- [Setting up Dynamic Reconfigure for a Node(cpp)](http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28cpp%29)
```
$ roslaunch ros_hello demo.launch --screen py:=0
```

- Set the configurable parameter **talker_test** in another terminal
```
$ rosrun dynamic_reconfigure dynparam set /ns1/talker talker_test true
...
[ INFO] [1590078139.911253829]: hello world 11
[ INFO] [1590078139.911795003]: I heard: [hello world 11]
[ INFO] [1590078140.911204146]: hello world 12 TEST
[ INFO] [1590078140.911761942]: I heard: [hello world 12 TEST]
```

### Simple Action Server and Client
- [actionlib](http://wiki.ros.org/actionlib)
    - Action Specification: Goal, Feedback, & Result
- [Работа с Action](http://docs.voltbro.ru/starting-ros/messaging/rabota-s-action.html)
```
$ roslaunch ros_hello action.launch --screen
...
process[action_server-2]: started with pid [20089]
process[action_client-3]: started with pid [20093]
Yay! The dishes are now cleanCurrent State: SUCCEEDED
[action_client-3] process has finished cleanly
...
```