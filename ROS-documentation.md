# Task 1
- Create a ros package with `catkin_create_pkg package_name [depend1] [depend2] ...`
- Write all the node files in `src` folder
- Source the right workspace
- Run `roscore` before running the nodes
- Run the nodes with
```bash
rosrun task1 publisher_node.cpp
rosrun task2 subscriber_node.cpp   # in a different terminal window
```
- Make sure to run `catkin_make` or `catkin build` while being in the `catkin_ws` folder which should contain the `src` folder

## Problems faced
- Not sourcing the right workspace after `catkin_make` or `catkin build`
- Not running `roscore` before starting the nodes

# Task 2
- To see the contents of a \*bag file, we need to subscribe to the topics that were recorded by it
- Used the `np.random.normal` method for creating random data
- Run the `add_noise` with 
```bash
rosrun add_noise add_noise.py
```

## Problems faced
- Not sourcing the right workspace
- Not setting the turtlebot model. Do it by running 
```bash
export TURTLEBOT3_MODEL=<model_name>   # waffle, burger, waffle-pi
```
- Running `catkin_make` or `catkin build` with wrong file structure of the package

---


# Course 1

## Ros Master
- Start ROS master with 
```bash
roscore
```

## Nodes
- Run a node with 
```bash
rosrun package_name node_name
```
- See active nodes with 
```bash
rosnode list
```
- Retrieve information about a node with 
```bash
rosnode info node_name
```

## Topics
- List active topics with 
```bash
rostopic list
```
- Subscribe and print the contents of a topic with 
```bash
rostopic echo /topic
```
- Show information about a topic with
```bash
rostopic info /topic
```

## ROS Messages
- Defined in \*.msg files
- See the type with
```bash
rostopic type /topic
```
- Publish a message to a topic with
```bash
rostopic pub /topic type data
```

## ROS Workspace Environment
- Default workspace: 
```bash
source /opt/ros/noetic/setup.bash
```
- Sourcing your catkin workspace
```bash
cd ~/catkin_ws
source ./devel/setup.bash # or source ./devel/setup.zsh
```
- Checking workspace path
```bash
echo $ROS_PACKAGE_PATH
```

## catkin build system
- Use `catkin build` instead of `catkin_make`
```bash
catkin build package_name
```
- Update environment every time you build a new package 
- Work is done in `src` folder
- To clean the build:
```bash
catkin clean
```
- Check catkin workspace with:
```bash
catkin config
```

## ROS launch
- Launch multiple nodes 
- Launches launch files which are written in XML as \*.launch
```bash
roslaunch file_name.launch
```
or 
```bash
roslaunch package_name file_name.launch
```

---
# Course 2

- ROS packages can contain source code, launch files, configuration files, message definitions, data, and documentation
- Good practice to separate message definitions package from other packages
- A package contains:
	- **config** folder (Parameter files (YAML))
	- **include/package_name** folder (C++ include headers)
	- **launch** folder (\*.launch files)
	- **src** folder (Source files)
	- **test** folder (Unit/ROS tests)
	- CMakeLists.txt (CMake build file)
	- package.xml (Package information)

##### package.xml
- defines the properties of the package
	- Package name
	- Version number
	- Authors
	- Dependencies on other packages
	- ...

##### CMakeLists.xml
1. Required CMake Version (`cmake_minimum_required()`)
2. Package Name (`project()`)
3. Configure C++ standard and compile features
4. Find other CMake/Catkin packages needed for build (`find_package()`)
5. Message/Service/Action Generators (`add_message_files()`, `add_service_files()`, `add_action_files()`)
6. Invoke message/service/action generation (`generate_messages()`)
7. Specify package build info export (`catkin_package()`)
8. Libraries/Executables to build (`add_library()` / `add_executable()`/`target_link_libraries()`)
9. Tests to build (`catkin_add_gtest()`)
10. Install rules(`install()`)
---
-  ROS main header file include
```cpp
#include <ros/ros.h>
```
- `ros::init(...)` has  to be called before other ROS functions
- `NodeHandle` is the access point for communications with ROS systems
- `ros::Rate` is a helper class to run loops at a desired frequency
- `ros::ok()` checks of a node should continue running
- `ROS_INFO()` logs messages to the filesystem
- `ros::spinOnce()` processes incoming messages via callbacks

#### Node handle
There are four main types of node handles
1. Default (public) node handle:
	`nh_ = ros::NodeHandle();`
2. Private node handle:
	`nh_private = ros::NodeHandle("~");`
3. Namespaced node handle:
	`nh_eth_ = ros::NodeHandle("eth");`
4. Global node handle (**NOT RECOMMENDED**):
	`nh_global_ = ros::NodeHandle("/");`
	
#### Logging
- Instead of `std::cout`, use `ROS_INFO`
```cpp
ROS_INFO("Result: %d", result); // printf style
ROS_INFO_STREAM("Result: " << result);
```

#### Subscriber
- When a message is received, callback function is called with  the contents of the message as argument
- Start listening to a topic by calling the method `subscribe()` of the node handle
```cpp
ros::Subscriber subscriber = nodeHandle.subscribe(topic, queue_size, callback_function);
```
- `ros::spin()` processes callbacks and will not return until the node has been shutdown

#### Publisher
- Create a publisher with the help of the node handle
```cpp
ros::Publisher publisher = nodeHandle.advertise<message_type>(topic, queue_size);
```
- Create the message contents
- Publish the contents with
```cpp
publisher.publish(message);
```

#### ROS parameter Server
- Nodes use the parameter server to store and retrieve parameters at runtime
- Best used for static data such as configuration parameters
- Parameters can be defined in launch files or separate YAML files
- LIst all parameters with
```cpp
rosparam list
```
- Get the value of parameters with
```cpp
rosparam get parameter_name
```
- Set the value of a parameter with
```cpp
rosparam set parameter_name value
```

#### Parameters
- Get a parameter in C++ with
```cpp
nodeHandle.getParam(parameter_name, variable)
```
- Method returns `true` if parameter was found, `false` otherwise

## RViz
- 3D visualisation tool for ROS
- Subscribes to topics and visualizes the message contents
- Interactive tools to publish user information
- Save and load setup as RViz configuration
- Extensible with plugins
- Run RViz with
```bash
 rviz
```
