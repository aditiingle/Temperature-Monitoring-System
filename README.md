# Intro Task for SC Robotics
The intro task I did for the SC Robotics Team.

The recently modified project mimics a real-time Temperature Monitoring System.
The names of the folders, nodes, topics, and classes were changed to reflect the actions they were carrying out.
The python node (temp_reader_node) now acts as a temperature reader. It generates random temperature values between 15.0 and 40.0 degrees Celsius.
The C++ node acts as the temperature monitor system. It takes the temperature measured by the python node over the temperature_topic topic and evaluates it. If the temperature reading exceeds 30 degrees Celsius then the Cpp node (temp_monitor_node) sends an alert to the python node over the alert_topic topic.

Path to the modified project: https://github.com/aditiingle/sc-robotics-intro-task/tree/modified-project/sc_intro_ws

These are the steps I followed to make the publisher and subscriber nodes in Python and C++ using ros2, and then to run the nodes :

Step 1 : Make a ROS2 Workspace for the project
  - Make the workspace folder in the home directory, intro_ws.
  - Inside the intro_ws folder, make an src folder where the packages are created.
  - Install colcon, and in the terminal, use colcon to build the intro_ws workspace.
  - in the .bashrc script add the line source ~/intro_ws/install/setup.bash to use the functionalities from the workspace.

Step 2 : Create a Python package
  - In the src folder of the intro_ws, create a Python package using the command: ros2 pkg create intro_py_pkg --build-type ament_python --dependencies rclpy .
    - The Python package is called intro_py_pkg.
    - The ament_python build-type will create a package structured for a Python node.
    - The rclpy dependency is added because the node will depend on the rclpy ros2 python library.
  - Open the src directory in vs code.
  - Use colcon to build the package, to not recieve the error message, downgrade to pip3 setuptools version 58.2.0. Then colcon build again.
    
Step 3 : Create a C++ package
- In the src folder of the intro_ws, create a C++ package using the command: ros2 pkg create intro_cpp_pkg --build-type ament_cmake --dependencies rclcpp .
    - The C++ package is called intro_cpp_pkg.
    - The ament_cmake build-type will create a package structured for a C++ node.
    - The rclcpp dependency is added because the node will depend on the rclpy ros2 C++ library.
  - Open the src directory in vs code.
  - Use colcon to build the package, the command: colcon build --package-select intro_cpp_pkg, will only build the C++ package.

Step 4 : Write a Python Node
  - The Python publisher-subscriber node is created in the intro_py_pkg folder of the intro_py_pkg folder in the src folder.
  - The command: chmod +x py_node.py makes the Python Node .py file an executable.
  - In the py_node file, the code for the Python publisher and the Python subscriber is written.
  - The Python publisher publishes on the py_topic topic. The timer is used to publish a message of message-type example_interfaces String to the topic at 2 Hz frequency.
  - The Python subscriber subscribes to the cpp_topic topic. It recieves the messages of type example_interfaces String from the C++ Node which publishes on the cpp_topic.
  - The example_interfaces dependency was added to the package.xml file.
  - The rclpy.spin function keeps the node alive until you press ctrl+C in the terminal.
  - In the setup.py file, specify the name of the executable that needs to be installed in the install folder, along with the main function to start execution from the main function of the node.
  - Compile the package using colcon build and use symlink-install.
  - When you run the package, the publisher will begin publishing on the py_topic, you can see this with the command: ros2 topic echo /py_topic while the node is still running.

Step 5 : Write a C++ Node 
  - The C++ publisher-subscriber node is created in the intro_cpp_pkg folder of the intro_cpp_pkg folder in the src folder.
  - The command: chmod +x cpp_node.cpp makes the C++ Node .cpp file an executable.
  - In the cpp_node file, the code for the C++ publisher and the C++ subscriber is written.
  - Include the rclcpp library in the .cpp program to be able to use the ros2 functionalities. Use C/C++: Edit Configurations to create a .vscode folder in the src directory, and in the c_cpp_properties.json file add a path which is the include folder of the global ros2 installation.
  - The C++ publisher publishes on the cpp_topic topic. The timer is used to publish a message of message-type example_interfaces String to the topic after every 500 milliseconds.
  - The C++ subscriber subscribes to the py_topic topic. It recieves the messages of type example_interfaces String from the Python Node which publishes on the py_topic.
  - The example_interfaces dependency was added to the package.xml file. And to the CMakeLists.txt file. It is also included in the cpp_node.cpp file.
  - The rclcpp::spin function keeps the node alive until you press ctrl+C in the terminal.
  - In the CMakeLists.txt file, add the executable with the name, cpp_node and link the dependencies with ament_target_dependencies.
  - In CMakeLists.txt, use install to the targets. It will install the node in the lib folder in the install folder of the ros2 workspace.
  - Compile the package using colcon build.
  - When you run the package, the publisher will begin publishing on the cpp_topic, you can see this with the command: ros2 topic echo /cpp_topic while the node is still running.

To see the messages being published and subscribed from both the nodes, run both nodes in separate terminals using the command: ros2 run intro_py_pkg py_node and the command: ros2 run intro_cpp_pkg cpp_node. The py_node will recieve and print the messages from the cpp_topic it subscribes to and the cpp_node will recieve and print the messages from the py_topic it subscribes to. To see the messages each node publishes to its topic, use the commands: ros2 topic echo /py_topic and ros2 echo /cpp_topic.
