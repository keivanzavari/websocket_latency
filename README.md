## Rosbridge communication latency

Package to measure the communication latency over rosbridge.

* Launch ros bridge using the provided launch file `launch/rosbridge.launch`. This instance of rosbridge watches the port `49152`.
* Run the two nodes, `publisher` and `subscriber`.
* Publisher node sends a message to the subscriber using websocket. You can run the subscriber on a different machine to measure the time it takes to deliver a message.
* Publisher has a dynamic reconfigure endpoint to change its frequency of sending data.

_Tested with ROS Noetic and Python 3.8.10 on Ubuntu 20.04 Focal._
