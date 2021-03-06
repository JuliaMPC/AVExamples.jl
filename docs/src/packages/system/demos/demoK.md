# demoK

In this demo, the vehicle follows a path autonomously, which is generated by `nloptcontrol_planner` and helps the vehicle to avoid obstacles. 
This demo utilizes the command `x`, `y`, `ux` from `nloptcontrol_planner`.
  
## To Run

```
$ roslaunch system demoK.launch
```

This may take a long time to initialize.

## Expected Output
The output of this demo should be similar to the figure below. `Running model for the: xxx time` shows the current step time. 

![link](demoK/demoK.png)

The relationship between different topics/topics is checked by opening a new terminal, and enter the docker by

```
$ docker exec -it <container_name> /bin/bash
```

`<container_name>` can be auto filled by the `Tab` key. Then, open the rqt graph by

```
$ rqt_graph
```

The output of `rqt_graph` is shown below. `nlopcontrol_planner/control` communicates with `/obstacle_avoidance`.

![link](demoK/demoK_rqt.png)


