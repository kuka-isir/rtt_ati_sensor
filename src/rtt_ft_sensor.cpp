#include "rtt_ati_sensor/rtt_ft_sensor.hpp"

rtt_ati::FTSensor::FTSensor(std::string const& name) : TaskContext(name){
    calibration_index_ = ati::current_calibration;
    ip_ = "";
    frame_ = rtt_ati::default_frame;

    this->addProperty("ip",ip_).doc("(xxx.xxx.xxx.xxx) The IP address for the ATI NET F/T box (default : "+ati::default_ip+" )");
    this->addProperty("frame",frame_).doc("(string) The name of the frame for the wrenchStamped msg (default : "+rtt_ati::default_frame+" )");
    this->addProperty("calibration_index", calibration_index_).doc("(uint) The calibration index to use (default: current)");

    this->ports()->addPort("WrenchStamped",this->port_WrenchStamped);
    port_WrenchStamped.createStream(rtt_roscomm::topic(this->getName()+"/wrench"));

    this->addOperation("setBias",&rtt_ati::FTSensor::setBias,this,RTT::OwnThread);
    this->addOperation("setBiasROS",&rtt_ati::FTSensor::setBiasROS,this,RTT::OwnThread);

    ft_sensor_ = boost::shared_ptr<ati::FTSensor>(new ati::FTSensor());
    set_bias_ = false;
}
bool rtt_ati::FTSensor::setBiasROS(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    set_bias_ = true;
    return true;
}

bool rtt_ati::FTSensor::setBias()
{
    set_bias_ = true;
    return true;
}

bool rtt_ati::FTSensor::configureHook(){
    bool configured = false;

    //  Setting the ROS Service set_bias
    boost::shared_ptr<rtt_rosservice::ROSService> rosservice = this->getProvider<rtt_rosservice::ROSService>("rosservice");

    if(rosservice)
        rosservice->connect("setBiasROS",this->getName()+"/set_bias","std_srvs/Empty");
    else
        RTT::log(RTT::Warning) << "ROSService not available" << RTT::endlog();

    //  Getting params from the parameter server,  if available
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!ip_.empty())
        RTT::log(RTT::Warning)<<"Orocos specified ip : " << ip_<<RTT::endlog();

    if(rosparam){
        configured = rosparam->getParam(getName() + "/ip","ip");
        if(!configured && ip_.empty())
        {
            ip_ = ati::default_ip;
            RTT::log(RTT::Warning)<<"ROSParam available, but couldn't get ip parameter, using default ip : " << ati::default_ip<<RTT::endlog();
        }else{
            RTT::log(RTT::Info)<<"ROSparam ip provided,  using " << ip_ <<RTT::endlog();
        }
        configured = rosparam->getParam(getName() + "/calibration_index","calibration_index");
        if(!configured && calibration_index_ ==  ati::current_calibration)
        {
            RTT::log(RTT::Warning)<<"ROSParam available, but couldn't get calibration index parameter, using current calibration " <<RTT::endlog();
        }else{
            RTT::log(RTT::Info)<<"ROSparam calibration index provided,  using " << calibration_index_ <<RTT::endlog();
        }
        configured = rosparam->getParam(getName() + "/frame","frame");
    }else if(ip_.empty()){
        ip_ = ati::default_ip;
        RTT::log(RTT::Warning)<<"ROSParam unavailable and no IP specified, using default ip : " << ati::default_ip<<RTT::endlog();
    }


    this->port_WrenchStamped.setDataSample(this->wrenchStamped);
    configured = ft_sensor_->init(ip_,calibration_index_,ati::command_s::REALTIME);
    
    double current_period = this->getActivity()->getPeriod();
    double current_rate = 0;
    if (current_period > 0)
        current_rate = 1 / current_period;
    int rdt_rate = ft_sensor_->getRDTRate();
    
    if (current_rate <= rdt_rate)
    {
      RTT::log(RTT::Warning)<<"Current component activity "<< current_rate << " Hz is lower than RDT output rate "<< rdt_rate << \
      " Hz and will cause lag in the data. Consider changing the component or the netft box settings" <<RTT::endlog();
    }
    
    this->wrenchStamped.header.frame_id = frame_;
    return configured;
}

void rtt_ati::FTSensor::updateHook(){
    ft_sensor_->getMeasurements(this->measurement,this->rdt_sequence,this->ft_sequence);
    lock_.unlock();
    this->wrenchStamped.header.stamp = rtt_rosclock::host_now();
    this->wrenchStamped.header.seq = this->ft_sequence;
    this->wrenchStamped.wrench.force.x = measurement[0];
    this->wrenchStamped.wrench.force.y = measurement[1];
    this->wrenchStamped.wrench.force.z = measurement[2];
    this->wrenchStamped.wrench.torque.x = measurement[3];
    this->wrenchStamped.wrench.torque.y = measurement[4];
    this->wrenchStamped.wrench.torque.z = measurement[5];
    this->port_WrenchStamped.write(wrenchStamped);
    if(set_bias_){
        set_bias_ = false;
        ft_sensor_->setBias();
    }
}

ORO_CREATE_COMPONENT(rtt_ati::FTSensor)
