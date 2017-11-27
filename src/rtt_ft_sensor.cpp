#include "rtt_ati_sensor/rtt_ft_sensor.hpp"
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_ros/rtt_ros.h>

rtt_ati::FTSensor::FTSensor(std::string const& name) : TaskContext(name){
    calibration_index_ = ati::current_calibration;
    ip_ = "";
    frame_ = rtt_ati::default_frame;
    read_mode_ = RD_MODE_USER_PERIOD;
    sample_count_ = rtt_ati::default_sample_count;

    this->addProperty("ip",ip_).doc("(xxx.xxx.xxx.xxx) The IP address for the ATI NET F/T box (default : "+ati::default_ip+" )");
    this->addProperty("read_mode",read_mode_).doc("(int) 0: user period, 1: event-based, 2: event-based at NetFT rate to match user periodicity, 3: user period matching NetFT rate (default : 0)");
    this->addProperty("frame",frame_).doc("(string) The name of the frame for the wrenchStamped msg (default : "+rtt_ati::default_frame+" )");
    this->addProperty("calibration_index", calibration_index_).doc("(uint) The calibration index to use (default: current)");
    this->addProperty("sample_count", sample_count_).doc("(int) number of samples, also determines the streaming mode (default: -1)");

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

    if(sample_count_ > 1)
        configured = ft_sensor_->init(ip_,calibration_index_,ati::command_s::BUFFERED, sample_count_);
    else
        configured = ft_sensor_->init(ip_,calibration_index_,ati::command_s::REALTIME, sample_count_);

    double current_period = this->getActivity()->getPeriod();
    double current_rate = 0;
    if (current_period > 0)
        current_rate = 1 / current_period;
        
    int rdt_rate = ft_sensor_->getRDTRate();
    
    // adapt period setting as requested
    switch (read_mode_)
    {
      case RD_MODE_EVENTBASED_SETRATE:
        if (rdt_rate != current_rate)
        {
          RTT::log(RTT::Warning) << "Changing netft RDT output rate to " << current_rate << " Hz" << RTT::endlog();
          if (ft_sensor_->setRDTOutputRate(current_rate))
          {
            rdt_rate = current_rate;
            // reinit the sensor to use the new setting
            if(sample_count_ > 1)
                configured = ft_sensor_->init(ip_,calibration_index_,ati::command_s::BUFFERED, sample_count_);
            else
                configured = ft_sensor_->init(ip_,calibration_index_,ati::command_s::REALTIME, sample_count_);
          }
          else
          {
            RTT::log(RTT::Warning) << "Could not set RDT output rate. Periodicity might be inadequate" << RTT::endlog();
          }
        }
        // set component activity to 0.0
        current_period = 0.0;
        this->getActivity()->setPeriod(current_period);
        
        if (sample_count_ != 0)
        {
          RTT::log(RTT::Warning) << "Event-based reading mode requested, but request sample count is not infinite, readout frequency will be incorrect" << RTT::endlog();
        }
        
        RTT::log(RTT::Info)<<"NetFT rate: " << rdt_rate <<RTT::endlog();
        break;
        
      case RD_MODE_EVENTBASED:
        if (current_period != 0.0)
        {
          RTT::log(RTT::Warning) << "Event-based reading mode requested, but component has a non zero periodicity, forcing zero periodicity" << RTT::endlog();
          current_period = 0.0;
          this->getActivity()->setPeriod(current_period);
        }
        
        if (sample_count_ != 0)
        {
          RTT::log(RTT::Warning) << "Event-based reading mode requested, but request sample count is not infinite, readout frequency will be incorrect" << RTT::endlog();
        }
        
        RTT::log(RTT::Info) << "NetFT rate: " << rdt_rate << RTT::endlog();
        break;
      
      case RD_MODE_USER2NETFT:
        if (current_rate < rdt_rate)
        {
          int new_rate = rdt_rate + 1;
          double new_period = 1.0 / new_rate;
          RTT::log(RTT::Warning) << "Setting component periodicity to " << new_period << " to be slightly higher than RDT output rate" << RTT::endlog();
          this->getActivity()->setPeriod(new_period);
          current_period = new_period;
        }
        else
        {
          RTT::log(RTT::Warning)<<"Periodicity already higher than RDT output rate. Not changing user periodicity" << RTT::endlog();
        }
        
      case RD_MODE_USER_PERIOD:
      default:
        if (sample_count_ == 0)
        {
          RTT::log(RTT::Warning) << "User triggered reading mode requested, but request sample count is 0 (infinite) and unknown behaviour is to be expected" << RTT::endlog();
          RTT::log(RTT::Info) << "NetFT rate: " << rdt_rate << RTT::endlog();
        }
        
        if (current_period == 0.0)
        {
          RTT::log(RTT::Warning) << "User triggered reading mode requested, but component periodicity is zero and unknown behaviour is to be expected" << RTT::endlog();
        }
        break;
    }
    
    this->wrenchStamped.header.frame_id = frame_;
    return configured;
}

void rtt_ati::FTSensor::updateHook(){
    ft_sensor_->getMeasurements(this->measurement,this->rdt_sequence,this->ft_sequence);
    lock_.unlock();
    this->wrenchStamped.header.stamp = rtt_rosclock::host_now();
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
    if (read_mode_ == RD_MODE_EVENTBASED || read_mode_ == RD_MODE_EVENTBASED_SETRATE)
      this->trigger();
}

ORO_CREATE_COMPONENT(rtt_ati::FTSensor)
