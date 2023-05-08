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





