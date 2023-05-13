# ROS2 Actions #
## Actions Vs Services ##
1) Actions are preemptable and can be stopped during execution
2) Actions provide feedback

## Actions use Services for handling the goal and the result and use Topics to handle the feedback ##


![](https://github.com/saurabhlanje/ROS2/blob/main/Actions/Action-SingleActionClient.gif)

## Workflow ##

    1) The Client sends a goal to the Server. This will trigger the "start" of the Action.
    2) The Server sends feedback to the Client while the Action is taking place.
    3) Once the Action finishes, the Server returns a response to the Client.
## ros2 action list ##
