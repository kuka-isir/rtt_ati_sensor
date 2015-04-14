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

/**
 * @brief Namespace containing all rtt related elements. Same as ati namespace for libati_sensor
 * 
 */
namespace rtt_ati{
/**
  * @brief The default output TF frame name
  * 
  */
static const std::string default_frame = "/ati_ft_link";
/**
 * @brief This class is a simple RTT wrapper around ati::FTSensor class from libati_sensor. 
 * 
 * It is used to communicate with the sensor at very high frequency (up to 7KHz) in hard realtime.
 * You get information using one orocos port (WrenchStamped msg) and set the bias using an Orocos operation (setBias)
 * The same are available through ROS (see https://github.com/kuka-lwr/rtt_ati_sensor) for more info).
 * You can also check https://github.com/kuka-lwr/ati_sensor for more info.
 * 
 * @author Antoine Hoarau <hoarau.robotics@gmail.com>
 */
class FTSensor : public RTT::TaskContext{
  public:
    FTSensor(std::string const& name);
    /**
     * @brief Get the parameters from rosparam, and tries to connect to the sensor
     * 
     * @return bool
     */
    bool configureHook();
    /**
     * @brief Get data from sensor and publishes a WrenchStamped msg to /ft_sensor/wrench topic
     * 
     * @return void
     */
    void updateHook();
    /**
     * @brief Set the bias for the sensor
     * 
     * @param request Empty
     * @param response Empty
     * @return bool
     */
    bool setBias(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
protected:
    /**
     * @brief The sensor shared pointer from libati_sensor
     * 
     */
    boost::shared_ptr<ati::FTSensor> ft_sensor_;
    /**
     * @brief The IP of the sensor
     * 
     */
    std::string ip_;
    /**
     * @brief The Orocos output port "wrench"
     * 
     */
    RTT::OutputPort<geometry_msgs::WrenchStamped> port_WrenchStamped;
    /**
     * @brief The ROS msg to be sent
     * 
     */
    geometry_msgs::WrenchStamped wrenchStamped;
    /**
     * @brief The measurement from the sensor (to be transformed to a WrenchStamped msg)
     * 
     */
    double measurement[6];
    /**
     * @brief The internal sample number of the F/T record contained in this RDT record. 
     * The F/T sequence number starts at 0 when the Net F/T is powered up and 
     * increments at the internal sample rate (7000 per sec). Unlike the RDT 
     * sequence number, ft_sequence does not reset to zero when an RDT request is 
     * received. The F/T sequence counter will roll over to zero for the increment 
     * following 4294967295 (2e32-1). See manual section 10.1
     * 
     */
    unsigned int ft_sequence;
    /**
     * @brief The position of the RDT record within a single output stream. The RDT
     * sequence number is useful for determining if any records were lost in transit.
     * For example, in a request for 1000 records, rdt_sequence will start at 1 and
     * run to 1000. The RDT sequence counter will roll over to zero for the
     * increment following 4294967295 (2e32-1). See manual section 10.1
     * 
     */
    unsigned int rdt_sequence;
    /**
     * @brief The calibration to use (-1 for current calibration). To be set on the sensor's webserver
     * 
     */
    int calibration_index_;
    /**
     * @brief The output frame name for rviz visualization (tf name)
     * 
     */
    std::string frame_;
};
}
#endif
