#include "rtt_ati_sensor/rtt_ft_sensor.hpp"

rtt_ati::FTSensor::FTSensor(std::string const& name) : TaskContext(name){
  this->addProperty("ip",ip_).doc("(xxx.xxx.xxx.xxx) The IP address for the ATI NET F/T box (default : "+ati::default_ip+" )");
  this->ports()->addPort("wrench",this->port_WrenchStamped);
  port_WrenchStamped.createStream(rtt_roscomm::topic(this->getName() +"/data"));
  this->addOperation("setBias",&rtt_ati::FTSensor::setBias,this,RTT::OwnThread);
  this->addProperty( "calibration_index", calibration_index_).doc("(uint) The calibration index to use (default: current)");
  calibration_index_ = ati::current_calibration;
  ip_ = "";
}
bool rtt_ati::FTSensor::setBias(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ft_sensor_->setBias();
    return true;
}

bool rtt_ati::FTSensor::configureHook(){
    bool all_configured = false;
    RTT::log().setLogLevel(RTT::Logger::Warning);
    
    boost::shared_ptr<rtt_rosservice::ROSService> rosservice = this->getProvider<rtt_rosservice::ROSService>("rosservice");
    rosservice->connect("setBias",this->getName()+"/set_bias","std_srvs/Empty");
    
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
    
    if(!ip_.empty())
        RTT::log(RTT::Warning)<<"Orocos specified ip : " << ip_<<RTT::endlog();
    
    if(rosparam){
        all_configured = rosparam->getComponentPrivate("ip");
        if(!all_configured && ip_.empty())
        {
            ip_ = ati::default_ip;
            RTT::log(RTT::Warning)<<"ROSParam available, but couldn't get ip parameter, using default ip : " << ati::default_ip<<RTT::endlog();
        }else{
            RTT::log(RTT::Info)<<"ROSparam ip provided : " << ip_ <<" ,using this."<<RTT::endlog();
        }
    }else if(ip_.empty()){
        ip_ = ati::default_ip;
        RTT::log(RTT::Warning)<<"ROSParam unavailable and no IP specified, using default ip : " << ati::default_ip<<RTT::endlog();
    }
    
    this->port_WrenchStamped.setDataSample(this->wrenchStamped);
    
    ft_sensor_ = boost::shared_ptr<ati::FTSensor>(new ati::FTSensor(ip_,calibration_index_));
    
    all_configured = ft_sensor_->init(ati::command_s::REALTIME);
    return all_configured;
}

void rtt_ati::FTSensor::updateHook(){
    ft_sensor_->getMeasurements(this->measurement,this->rdt_sequence,this->ft_sequence);
    this->wrenchStamped.header.stamp = rtt_rosclock::host_now();
    this->wrenchStamped.header.seq = this->ft_sequence;
    this->wrenchStamped.wrench.force.x = measurement[0];
    this->wrenchStamped.wrench.force.y = measurement[1];
    this->wrenchStamped.wrench.force.z = measurement[2];
    this->wrenchStamped.wrench.torque.x = measurement[3];
    this->wrenchStamped.wrench.torque.y = measurement[4];
    this->wrenchStamped.wrench.torque.z = measurement[5];
    this->port_WrenchStamped.write(wrenchStamped);
}

ORO_CREATE_COMPONENT(rtt_ati::FTSensor)
