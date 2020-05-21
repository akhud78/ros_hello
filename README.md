# ros_hello
ROS HelloWorld

- Writing a Simple Publisher and Subscriber
- Writing a Simple Service and Client
- Building and running tests (gtest)

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
### C++
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
```
- Stop roscore in first terminal
```
[ INFO] [1589803053.553447324]: hello world 39
[ INFO] [1589803054.053424323]: hello world 40
$ fg
roscore
^C[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```
- [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
```
$ rosrun ros_hello add_two_ints_server
$ rosservice list
/add_two_ints
$ rosservice args /add_two_ints
a b
$ rosservice call /add_two_ints 1 2
sum: 3
```
### Python
- [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- Run talker.py
```
$ rosrun ros_hello talker.py
[INFO] [1589808962.491005]: hello python world 1589808962.49
[INFO] [1589808963.492489]: hello python world 1589808963.49
```
- Run listener node (C++) in another terminal
### Building and running tests (gtest)
- [Building and running tests](http://wiki.ros.org/gtest)
```
$ roscore &
$ rosrun ros_hello hello_test_utils
...
[==========] 1 test from 1 test case ran. (1 ms total)
[  PASSED  ] 1 test.
```
