This package has to be a CMake package. Currently, there is no way to generate custom interfaces in a pure Python package.
However, you can create a custom interface in a CMake package and then use it in a Python node.
### Create pkg ###
```ros2 pkg create --build-type ament_cmake custom_interfaces --dependencies rclcpp std_msgs```

Specify this is a CMake package with the --build-type flag set to ament_cmake


Once you have created this new package, you will create a new message.

To create a new interface, you have to follow the next steps:

    Create a directory named msg inside your package
    Inside this directory, create a file named name_of_your_message.msg (more - information below)
    Modify the CMakeLists.txt file (more information below)
    Modifypackage.xml file (more information below)
    Compile and source
    Use in your node

