# twist_to_ackermann
Ros 2 Humble package providing a node that converts twist commands for a differential drive robot to Ackermann steering commands. 

## systemd
```
./systemd/service_setup.sh install
```
installs a systemd service on the host which launches the node automatically inside the docker container.
It assumes the docker container and the ROS package are installed appropriately.