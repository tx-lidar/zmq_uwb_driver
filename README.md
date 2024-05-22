***
This project is used to parse the zmq protocol, subscribe to JSON message data with specified keywords, parse it, and finally publish the parsed data through custom ROS messages
***

# system
ubuntu20.04  ros1  

# catkin
```
mkdir -p catkin_ws/src 
```
```
cd catkin_ws/src
```
```
git clone git@github.com:tx-lidar/zmq_uwb_driver.git
```
```
cd ..
```
```
catkin_make
```

# run
```
roslaunch zmq_uwb_driver zmq_uwb_driver.launch
```

