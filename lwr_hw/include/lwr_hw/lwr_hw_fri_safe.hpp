#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRI remote hooks
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include <thread>
#include "fri/friudp.h"
#include "fri/friremote.h"

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

class LWRHWFRI : public LWRHW
{

public:

  LWRHWFRI() : LWRHW() { }

  ~LWRHWFRI() { stopKRCComm_ = true; KRCCommThread_.get()->join();}

  void setPort(int port){port_ = port; port_set_ = true;};
  void setIP(std::string hintToRemoteHost){hintToRemoteHost_ = hintToRemoteHost; ip_set_ = true;};
  float getSampleTime(){return sampling_rate_;};
  // This function sets the emergency event flag
  bool setEmergencyEvent(bool eevent)
  {
      std::swap(eevent_,eevent);
      if(!eevent)
      {
          emergency_scaling_factor = 1.0;
      }
      return eevent;
  };
  bool setAllowPositionControl(bool b_in){std::swap(allow_position_control_,b_in); return b_in;}; // This function sets the allow position control flag
  
  // Init, read, and write, with FRI hooks
  bool init()
  {
    if( !(port_set_) || !(ip_set_) )
    {
      std::cout << "Did you forget to set the port/ip?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
      return false;
    }

    // construct a low-level lwr
    device_.reset( new friRemote( port_, const_cast<char*>(hintToRemoteHost_.c_str()) ) );

    // initialize FRI values
    lastQuality_ = FRI_QUALITY_BAD;
    lastCtrlScheme_ = FRI_CTRL_OTHER;

    std::cout << "Opening FRI Version " 
      << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
      << " Interface for LWR ROS server" << std::endl;

    std::cout << "Checking if the robot is Stopped..." << std::endl;
    if( device_->getState() == FRI_STATE_OFF )
    {
      std::cout << "Please, start the KRL script now." << std::endl;
    }
    KRCCommThread_.reset( new std::thread( &LWRHWFRI::KRCCommThreadCallback,this ) );

    startFRI();

    std::cout << "Ready, FRI has been started!" << std::endl;
    std::cout << "FRI Status:\n" << device_->getMsrBuf().intf << std::endl;
    sampling_rate_ = device_->getSampleTime();
    std::cout << "Sampling Rate: " << sampling_rate_ << std::endl;
    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = device_->getMsrMsrJntPosition()[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = device_->getMsrJntTrq()[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    for(int j = 0; j < 12; j++)
    {
        cart_pos_[j] = device_->getMsrCartPosition()[j];
    }
    for(int j = 0; j < 6; j++)
    {
        cart_stiff_[j] = cart_stiff_command_[j];
        cart_damp_[j] = cart_damp_command_[j];
        cart_wrench_[j] = cart_wrench_command_[j];
    }
    return;
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    float newJntPosition[n_joints_];
    float newJntStiff[n_joints_];
    float newJntDamp[n_joints_];
    float newJntAddTorque[n_joints_];
    float newCartPos[12];
    float newCartStiff[6];
    float newCartDamp[6];
    float newAddFT[6];

    switch (getControlStrategy())
    {
      case CARTESIAN_IMPEDANCE:
        // This is the reaction to an emergency event. For now this sets all variables (stiffness, damping, and ext_force) to zero, position to the last commanded cartesian position
        if(eevent_)
        {
            for(int i=0; i < 12; ++i)
            {
                newCartPos[i] = device_->getMsrCartPosition()[i];
            }
            for(int i=0; i < 6; i++)
            {
                newCartStiff[i] = emergency_scaling_factor*cart_stiff_command_[i];
                newCartDamp[i] = 0.0;
                newAddFT[i] = 0.0;
            }
            emergency_scaling_factor *= 0.98175; // from 1000.0 to 0.1 in 500 runs
        }
        else
        {
            for(int i=0; i < 12; ++i)
            {
                newCartPos[i] = cart_pos_command_[i];
            }
            for(int i=0; i < 6; i++)
            {
                newCartStiff[i] = cart_stiff_command_[i];
                newCartDamp[i] = cart_damp_command_[i];
                newAddFT[i] = cart_wrench_command_[i];
            }
        }
        device_->doCartesianImpedanceControl(newCartPos, newCartStiff, newCartDamp, newAddFT, NULL, false);
        break;

        
      case JOINT_POSITION:
        for (int j = 0; j < n_joints_; j++)
        {
            newJntPosition[j] = joint_position_command_[j];
            joint_set_point_command_[j] = joint_position_command_[j];
        }
        if(allow_position_control_)
        {
            device_->doPositionControl(newJntPosition, false);
            break;
        }
        else
        {
            // do joint_impedance control instead...
        }

      case JOINT_IMPEDANCE:
        if(eevent_)
        {
            for(int j=0; j < n_joints_; j++)
            {
                // This is the reaction to an emergency event. For now this sets all variables (stiffness, damping, and ext_torque) to zero, position to the last commanded joint position
                newJntAddTorque[j] = 0.0;
                // using an extra variable because the command could increase with time, in principle faster than the scaling factor
                newJntStiff[j] = emergency_scaling_factor*joint_stiffness_command_[j];
                newJntDamp[j] = 0.7;
            }
            emergency_scaling_factor *= 0.98175; // from 1000.0 to 0.1 in 500 runs
            device_->getCurrentCmdJntPosition(newJntPosition); // both position command and offset
        }
        else
        {
            for(int j=0; j < n_joints_; j++)
            {
                newJntPosition[j] = joint_set_point_command_[j];
                newJntAddTorque[j] = joint_effort_command_[j];
                newJntStiff[j] = joint_stiffness_command_[j];
                newJntDamp[j] = joint_damping_command_[j];
            }
        }
        
        device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, false);
        break;

    }
    return;
  }

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    // at this point, we now that there is only one controller that ones to command joints
    ControlStrategy desired_strategy = JOINT_IMPEDANCE; // default

    desired_strategy = getNewControlStrategy(start_list,stop_list,desired_strategy);

    if(JOINT_POSITION == desired_strategy && !allow_position_control_)
    {
        std::cout << "You are no longer allowed to use Position Control. Using instead Joint Impedance Control. Safety first!" << std::endl;
    }

    for (int j = 0; j < n_joints_; ++j)
    {
      ///semantic Zero
      joint_position_command_[j] = joint_position_[j];
      joint_effort_command_[j] = 0.0;

      ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
      try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}
      try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}

      ///reset joint_limit_interfaces
      pj_sat_interface_.reset();
      pj_limits_interface_.reset();
    }

    if(desired_strategy == getControlStrategy())
    {
      std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
      stopFRI();

      // send to KRL the new strategy
      if( desired_strategy == JOINT_POSITION && allow_position_control_ )
        device_->setToKRLInt(0, JOINT_POSITION);
      else if( desired_strategy == JOINT_POSITION && !allow_position_control_ )
        device_->setToKRLInt(0, JOINT_IMPEDANCE);
      else if( desired_strategy == JOINT_IMPEDANCE)
        device_->setToKRLInt(0, JOINT_IMPEDANCE);
      else if( desired_strategy == CARTESIAN_IMPEDANCE)
        device_->setToKRLInt(0, CARTESIAN_IMPEDANCE);


      startFRI();

      setControlStrategy(desired_strategy);
      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
      if(!allow_position_control_ && desired_strategy == JOINT_POSITION)
          std::cout << "ATTENTION: in the safe interface, switching to JOINT_POSITION is not allowed! Using JOINT_IMPEDANCE on the robot instead, your commands will be forwarded..." << std::endl;
    }
  }

private:

  // Parameters
  int port_;
  bool port_set_ = false;
  std::string hintToRemoteHost_;
  bool ip_set_ = false;
  bool eevent_ = false; // Variable to manage emergency events, false by default
  bool allow_position_control_ = true;
  double emergency_scaling_factor; // Scale the stiffness in case of an emergency event, making it smooth to transition from the current to a 0 stiffness value

  // low-level interface
  boost::shared_ptr<friRemote> device_;

  // FRI values
  FRI_QUALITY lastQuality_;
  FRI_CTRL lastCtrlScheme_;

  float sampling_rate_;

  boost::shared_ptr<std::thread> KRCCommThread_;
  bool stopKRCComm_ = false;
  void KRCCommThreadCallback()
  {
    while(!stopKRCComm_)
    {
      device_->doDataExchange();
    }
    return;
  }

  void startFRI()
  {
    // wait until FRI enters in command mode
    // std::cout << "Waiting for good communication quality..." << std::endl;
    // while( device_->getQuality() != FRI_QUALITY_OK ){};
    device_->setToKRLInt(1, 1);
    device_->doDataExchange();

    // std::cout << "Waiting for command mode..." << std::endl;
    // while ( device_->getFrmKRLInt(1) != 1 )
    // {
      // std::cout << "device_->getState(): " << device_->getState() << std::endl;
      // device_->setToKRLInt(1, 1);
      // usleep(1000000);
    // }
    return;
  }

  void stopFRI()
  {
    // wait until FRI enters in command mode
    device_->setToKRLInt(1, 0);
    std::cout << "Waiting for monitor mode..." << std::endl;
    while ( device_->getFrmKRLInt(1) != 0 ){}
    // {
      // std::cout << "device_->getState(): " << device_->getState() << std::endl;
      // std::cout << "Waiting for monitor mode..." << std::endl;
      // device_->setToKRLInt(1, 0);
      // usleep(1000000);
    // }
    return;
  }

};

}

#endif
