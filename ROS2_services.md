### Client: who calls the service ###

### Server: who serves the call ###

### List all currently active services ###

```ros2 service list```

### Call service from CLI ###

```ros2 service call <service_name> <service_type> <value>```

### Service type ###
Find what kind of argument to be passed to the service
```ros2 service type /moving```

```ros2 service type <service_name>```

### Format of argument to be passed ###
```ros2 interface show <type of service>```
It is provided in the form like thos
```
<request argument>
---
<response argument>
```

### Write service client ###
#### client.py ####
```
# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_client
        super().__init__('service_client')
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(Empty, 'moving')
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.req = Empty.Request()
        

    def send_request(self):
        
        # send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'the robot is moving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Launch file ####

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='client_pkg',
            executable='service_client',
            output='screen'),
    ])

```

#### Setup file ####

```
from setuptools import setup
import os
from glob import glob

package_name = 'client_pkg'

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
        'service_client = client_pkg.service_client:main',
        ],
    },
)
```

### Service can be called in two ways ###

#### Synchronous call ####
A synchronous client will block the calling thread when sending a request to a service until a response has been received; nothing else can happen on that thread during the call. The call can take arbitrary amounts of time to complete. Once complete, the response returns directly to the client.

#### Asynchronous calls ####
Async calls in rclpy are entirely safe and the recommended method of calling services. They can be made from anywhere without running the risk of blocking other ROS and non-ROS processes, unlike sync calls.

It is not recommended to implement a synchronous service client. They are susceptible to deadlock, but will not provide any indication of issue when deadlock occurs. If you must use synchronous calls, the example in section 1 Synchronous calls is a safe method of doing so. You should also be aware of the conditions that cause deadlock outlined in section 1.1 Sync deadlock. We recommend using async service clients instead.


