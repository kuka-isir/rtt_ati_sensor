// ISIR 2015 Antoine Hoarau <hoarau.robotics@gmail.com>
#ifndef OROCOS_RTT_ATI_SENSOR_COMPONENT_HPP
#define OROCOS_RTT_ATI_SENSOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <ati_sensor/ft_sensor.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_ros/rtt_ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <rtt_roscomm/rosservice.h>

namespace rtt_ati{
static const std::string default_frame = "/ati_ft_link";
class FTSensor : public RTT::TaskContext{
  public:
    FTSensor(std::string const& name);
    bool configureHook();
    void updateHook();
    bool setBias(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
protected:
  boost::shared_ptr<ati::FTSensor> ft_sensor_;
  std::string ip_;
  RTT::OutputPort<geometry_msgs::WrenchStamped> port_WrenchStamped;
  geometry_msgs::WrenchStamped wrenchStamped;
  double measurement[6];
  unsigned int ft_sequence,rdt_sequence;
  int calibration_index_;
  std::string frame_;
};
}
#endif
