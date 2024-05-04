
//
// Created by qiayuan on 1/24/22.
//

#include "legged_sirius_hw/SiriusHW.h"

namespace legged {
SiriusHW::SiriusHW():
    cheetah2ROS("udpm://239.255.76.67:7667?ttl=255"),
    ROS2cheetah("udpm://239.255.76.67:7667?ttl=255")
    {LeggedHW();}

bool SiriusHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

// TODO initialize LCM

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
  cheetah2ROS.subscribe("cheetah2ROS",&SiriusHW::handleROS2CheetahLCM,this);
  recv_thread = std::thread([&]() { for(;;) cheetah2ROS.handle(); });
  if (robot_type == "sirius_mini") {
  }
  else if(robot_type == "sirius_belt")
  {
  }
  else if(robot_type == "sirius_mid")
  {
  }
  else if(robot_type == "sirius_mid_with_arm")
  {
  }
  else {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }
  return true;
}

void SiriusHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < 12; ++i) {
    if(i==0||i==3||i==6||i==9)
    {
      jointData_[i].pos_ = ros_lowstate.q[i];
      jointData_[i].vel_ = ros_lowstate.qd[i];
    }
    else
    {
      jointData_[i].pos_ = -1 *ros_lowstate.q[i];
      jointData_[i].vel_ = -1 *ros_lowstate.qd[i];
    }

    // std::cout<<jointData_[i].pos_<<"<--->";
  }
    // std::cout<<std::endl;
  imuData_.ori_[0] = ros_lowstate.quat[1];
  imuData_.ori_[1] =ros_lowstate.quat[2];
  imuData_.ori_[2] = ros_lowstate.quat[3];
  imuData_.ori_[3] = ros_lowstate.quat[0];
  imuData_.angularVel_[0] = ros_lowstate.omegaBody[0];
  imuData_.angularVel_[1] = ros_lowstate.omegaBody[1];
  imuData_.angularVel_[2] = ros_lowstate.omegaBody[2];
  imuData_.linearAcc_[0] = ros_lowstate.aBody[0];
  imuData_.linearAcc_[1] = ros_lowstate.aBody[1];
  imuData_.linearAcc_[2] = ros_lowstate.aBody[2];
  // std::cout<<imuData_.linearAcc_[0]<<"<--->";
  //   std::cout<<imuData_.linearAcc_[1]<<"<--->";
  //     std::cout<<imuData_.linearAcc_[2]<<"<--->";
  // std::cout<<std::endl;

  // No contact feedback.
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] =0;
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void SiriusHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // LCM send

  for (int i = 0; i < 12; ++i) {
    if(i==0||i==3||i==6||i==9)
    {
      ros_lowcmd.q[i] = static_cast<float>(jointData_[i].posDes_);
      ros_lowcmd.qd[i] = static_cast<float>(jointData_[i].velDes_);
      ros_lowcmd.tau_ff[i] = static_cast<float>(jointData_[i].ff_);    
    }
    else
    {
      ros_lowcmd.q[i] = -1 *static_cast<float>(jointData_[i].posDes_);
      ros_lowcmd.qd[i] = -1 *static_cast<float>(jointData_[i].velDes_);
      ros_lowcmd.tau_ff[i] = -1 * static_cast<float>(jointData_[i].ff_);
    }

    // std::cout<<ros_lowcmd.q[i]<<"<-->";
    // std::cout<<ros_lowcmd.tau_ff[i]<<"<-->";
  }
  // std::cout<<std::endl;
    ROS2cheetah.publish("ROS2cheetah",&ros_lowcmd);
}

bool SiriusHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = 0;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = 1;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = 2;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = 3;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool SiriusHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool SiriusHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

void SiriusHW::handleROS2CheetahLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const ros_lowstate_lcmt* msg)
{
    (void)rbuf;
    (void)chan;
    memcpy(&ros_lowstate, msg, sizeof(ros_lowstate));
}                                  
}  // namespace legged
