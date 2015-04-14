# RTT ATI Sensor

Simple Orocos RTT wrapper for the ati_sensor library.

### Usage 

The best way to launch the sensor is to first start a roscore and set the sensor's parameters : 

```bash
roscore
# Set the sensor's ip address : 
rosparam set /ft_sensor/ip "192.168.100.103"
# (Optional) Set the calibration index you want to use (don't set for current calibration)
rosparam set /ft_sensor/calibration_index 1
```
Then you can launch the sensor using the deployer : 
```bash
deployer -s $(rospack find rtt_ati_sensor)/scripts/ft_sensor.ops
```

Or in a launch file : 
```xml
<launch>
 <arg name="ip" default="192.168.100.103"/>
  <node name="ft_sensor" pkg="rtt_ros" type="deployer" args="-s $(find rtt_ati_sensor)/scripts/ft_sensor.ops --" output="screen">
  <param name="ip" value="$(arg ip)"/>
</node>
</launch>
```


#### Integrate the Sensor in your OPS file
```lua
import("rtt_ros")
ros.import("rtt_rospack")
runScript(ros.find("rtt_ati_sensor") + "/scripts/ft_sensor.ops")
```

