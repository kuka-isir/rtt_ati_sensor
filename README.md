[![Build Status](https://travis-ci.org/kuka-isir/rtt_ati_sensor.svg?branch=master)](https://travis-ci.org/kuka-isir/rtt_ati_sensor)
# RTT ATI Sensor

Simple Orocos RTT wrapper for the ati_sensor library. It uses ROS to get parameters from the parameter server and ROS services to reset the bias.

###### OROCOS Port 
* ***WrenchStamped*** (geometry_msgs/WrenchStamped): 6D data from the F/T sensor

###### Parameters
* ***ip*** (string, default : 192.168.100.103) : The ip of the sensor 
* ***calibration_index*** (integer, default : -1 [current]): The calibration file to use 
* ***frame*** (string, default : /ati_ft_link): The output frame for rviz visualization 

###### Services 
* ***set_bias*** (std_msgs/Empty): Reset the software bias 

###### Topics 
* ***wrench*** (geometry_msgs/WrenchStamped): 6D data from the F/T sensor

> Note : The sequence number is ft_sequence (see the official [manual](http://www.ati-ia.com/app_content/documents/9620-05-NET%20FT.pdf) for more info) 

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

You'll have to set the parameters using rosparam (example using a launch file):
```xml
<launch>
<rosparam ns="ft_sensor" subst_value="true">
        ip: 192.168.100.103
        calibration_index: -1 # -1 for current
        frame: /ati_ft_link
 </rosparam>
 </launch>
```
And add the following lines to you ops script : 

```lua
import("rtt_ros")
ros.import("rtt_rospack")
runScript(ros.find("rtt_ati_sensor") + "/scripts/ft_sensor.ops")
```


### OPS file options

Two data reading mode are possible: ***User-based trigger*** and ***Event-based trigger***

By default (no parameters given), ***User-based trigger*** is used with given periodicity

#### User-based trigger

The data is queried at the rate of the component set in the ***setActivity*** command

* optional ***ft_sensor.sample_count = 1***
* ***ft_sensor.read_mode = 0*** for given periodicity 
* or ***ft_sensor.read_mode = 3*** to match the periodicity to the NetFT output rate.
 
#### Event-based trigger

The data is output at the rate of the NetFT output rate setting. 

This mode should have the smallest delay in the data due to continuous reading.

* ***ft_sensor.sample_count = 0*** (means infinite number of samples)
* ***ft_sensor.read_mode = 1*** for current NetFT rate 
* or ***ft_sensor.read_mode = 2*** to set NetFT to given component period via ***setActivity*** (component period is then modified internally to 0.0).

Note, NetFT rate can only be values that 7000 / integer can reach (7000, 3500, 2333, 1750, 1400, 1167, 1000, ...)
