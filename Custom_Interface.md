# Create custom interface #
This package has to be a CMake package. Currently, there is no way to generate custom interfaces in a pure Python package.
However, you can create a custom interface in a CMake package and then use it in a Python node.
### 1) Create pkg ###
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


### 2) Create a directory msg in your package.  ###
### 3) Create file within msg folder with name Age.msg with following content ###
```
int32 year
int32 month
int32 day
```

### 4) Add this line to CMakeLists ###
```find_package(rosidl_default_generators REQUIRED)```
And
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)
```

### 5) Add these lines to package.xml ###
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### 6) Build the package and source the setup file ###

### 7) Check msg interface is working or not ###
```ros2 interface show custom_interfaces/msg/Age```

# Use custom interface #
## 1) Make new package with dependencies consisting of package made earlier which consist of custom message ##
```ros2 pkg create --build-type ament_python example36_pkg --dependencies rclpy std_msgs geometry_msgs custom_interfaces```

## 2) Define dependencies of custom msg package in CMakeList ##
```<depend>custom_interfaces</depend>```

## 3) Node template ##
```
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Import custom interface Age
from custom_interfaces.msg import Age


class Example36(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('example36')
        # create the publisher object
        self.publisher_ = self.create_publisher(Age, 'age', 10)
        # create an Age message
        self.age = Age()
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # create an Age message
        self.age.year = 2031
        self.age.month = 5
        self.age.day = 21
        # publish the Age message
        self.publisher_.publish(self.age)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    example36 = Example36()
    rclpy.spin(example36)
    # Explicity destroy the node
    example36.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
## 4) Make launch file ##
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example36_pkg',
            executable='example36',
            output='screen'),
    ])
```
## 5) Modify setup.py ##
```
from setuptools import setup
import os
from glob import glob

package_name = 'example36_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'example36 = example36_pkg.example36:main'
        ],
    },
)
```


