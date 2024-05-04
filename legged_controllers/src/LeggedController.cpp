//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ocs2_core/Types.h>
int start;
int start_print;
// namespace {
// vector_t DEFAULT_JOINT_STATE(15);
// }
namespace legged {
vector_t DEFAULT_JOINT_STATE(15);
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  const std::string robotName = "legged_robot";
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);

  // pos_desired = vector_t::Zero(2);
  // pos_current = vector_t::Zero(2);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));
  std::string gaitCommandFile;
  controller_nh.getParam("/gaitCommandFile", gaitCommandFile);
  loadData::loadStdVector(gaitCommandFile, "list", gaitList_, verbose);
  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitCommandFile, gaitName, verbose)});
  }
  lastGaitCommand_ = "stance";
  GaitCommand_ = "trot";
  // std::string gaitCommandFile;
  // controller_nh.getParam("/gaitCommandFile", gaitCommandFile);
  // GaitKeyboardPublisher gaitCommand(controller_nh, gaitCommandFile, robotName, true);
  // ModeSequenceTemplate modeSequenceTemplate = gaitCommand.gaitMap_rc.at("stance");
  gait_pub = nh.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);                                                          
  auto joyCallback = [this](const sensor_msgs::Joy::ConstPtr& msg) {
    if (msg->axes[5]==-1){
    this->damping=true;
    }
    start=msg->buttons[5];
    start_print=msg->buttons[4];
    // vector_t error = pos_desired - pos_current;
    // if (error.squaredNorm()<0.004 && lastGaitCommand_ != "stance"){
    // // GaitCommand_ = "stance";
    // ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("stance");
    // gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
    // lastGaitCommand_ = "stance";
    // }
    // else if (error.squaredNorm()>=0.004 && lastGaitCommand_ != "static_walk"){
    // // GaitCommand_ = "bipedal_gait";
    // ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at("static_walk");
    // gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
    // lastGaitCommand_ = "static_walk";
    // }
  };
  // modeSequenceTemplate = gaitMap_.at("stance");
  sub=nh.subscribe<sensor_msgs::Joy>("/joy",10,joyCallback);
  // gait_pub = nh.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);
  // Hardware interface

  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                       "joint1", "joint2", "joint3"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  // wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
  //                                      *eeKinematicsPtr_);
  // wbc_->loadTasksSetting(taskFile, verbose);


  const std::string& eeName = "RH_FOOT";
  TailKinematicsPtr_ = leggedInterface_->getEeKinematicsPtr({eeName}, eeName);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
  
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  controller_time=time.toSec();
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    
    mpcMrtInterface_->advanceMpc();

    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();

  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);
  ros::NodeHandle nodeHandle;

  // Update the current state of the system
  
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  // mpcMrtInterface_->
  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();
  // mpcMrtInterface_->getTargetTrajectories().stateTrajectory;
  // pos_current = currentObservation_.state.segment<2>(6);
  // pos_desired = mpcMrtInterface_->getReferenceManager().getTargetTrajectories().stateTrajectory[1].segment<2>(6);
  // modeSequenceTemplate = gaitMap_.at("stance");
  // gait_pub.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));

  // std::cout<<mpcMrtInterface_->getReferenceManager().getTargetTrajectories().stateTrajectory[1]<<std::endl;
  // std::cout<<"  "<<std::endl;
  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
  // Whole body control
  currentObservation_.input = optimizedInput;
  
  // const vector_t currentjoint_state = currentObservation_.state.segment<12>(12);
  // std::cout<<currentjoint_state<<std::endl;
  // std::cout<<"  "<<std::endl;

  // wbcTimer_.startTimer();
  // vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  // wbcTimer_.endTimer();

  vector_t rbd_torque;
  CentroidalModelRbdConversions RbdConversions(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo());  
  const vector_t joint_accleration = vector_t::Zero(15);
  rbd_torque = rbdConversions_->computeRbdTorqueFromCentroidalModel(currentObservation_.state,optimizedInput,joint_accleration);
  

  vector_t torque = rbd_torque.tail(15);
  // vector_t tailForce = centroidal_model::getContactForces(currentObservation_.input, 4, leggedInterface_->getCentroidalModelInfo());
  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  // std::cout<<"force"<<std::endl;
  // std::cout<<tailForce<<std::endl;
  // if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
  //   ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
  //   stopRequest(time);
  // }
  // posDes=DEFAULT_JOINT_STATE;
  // velDes=vector_t::Zero(15);
  // velDes.tail(3)=vector_t::Zero(15);
  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    if (this->damping==true){
      hybridJointHandles_[j].setCommand(0, 0, 0, 3, 0);
    }
    else{
      if (j<12){
      // hybridJointHandles_[j].setCommand(posDes(j), 0, 15, 1, torque(j));
      // hybridJointHandles_[j].setCommand(posDes(j), 0, 50, 2, 0);
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 2, torque(j));
      // hybridJointHandles_[j].setCommand(posDes(j), 0, 150, 3, 0);
      }
      else{
      // hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 15, 0.5, torque(j));
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 15, 0.5, torque(j));
      // hybridJointHandles_[j].setCommand(posDes(j), 0, 150, 3, 0);
      }
      // hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 100, 2, 0);
    //   }
    //   else
    //   {
    //     hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 2, torque(j));
    //   }
    // }
  }
  }

  // if (start_print==1){

  // eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  // const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  // auto& data = leggedInterface_->getPinocchioInterface().getData();
  // size_t actuatedDofNum = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;

  // vector_t qPino(leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
  // vector_t vPino(leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
  // qPino.segment<3>(0)=measuredRbdState_.segment<3>(3);
  // qPino.segment<3>(3) = measuredRbdState_.head<3>(); 
  // qPino.segment<12>(7) = measuredRbdState_.segment<12>(6);
  // qPino(6)=measuredRbdState_(18);
  // vPino.setZero();
 
  // pinocchio::forwardKinematics(model, data, qPino, vPino);
  // pinocchio::updateFramePlacements(model, data);
  // Eigen::VectorXd q = randomConfiguration(model);
  // vector_t q=measuredRbdState_.segment(6, actuatedDofNum);
  // crba(model,data,qPino);
  // vector_t com_P=getComFromCrba(model, data);

  // const auto eePos = eeKinematicsPtr_->getPosition(vector_t());
  // std::cout<<"footname="<<std::endl;
  // std::cout<<eePos[3]<<std::endl;
  // TailKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  // vector_t pose = TailKinematicsPtr_->getPosition(currentObservation_.state).front();
  // std::cout<<TailKinematicsPtr_->getIds()[0]<<std::endl;
  // std::cout<<pose<<std::endl;
  // for (int i=0;i<5;i++){
  // std::cout<<leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices[i]<<std::endl;
  // std::cout<<"  "<<std::endl;
  // }
  // std::vector<vector3_t> positions;
  // for (int i=0;i<4;i++){
  // positions.emplace_back(data.oMf[leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices[i]].translation());
  // }
  // for (int i=0;i<4;i++){
  // std::cout<<positions[i]<<std::endl;
  // }
  // std::cout<<"com="<<std::endl;
  // std::cout<<com_P<<std::endl;
  // std::cout<<"  "<<std::endl;
  // for(auto name : leggedInterface_->getPinocchioInterface().getModel().names)
  //           std::cerr<<name<<"\n";
  // std::cout<<leggedInterface_->getPinocchioInterface().getModel().names[0]<<std::endl;
  // std::cout<<leggedInterface_->getCentroidalModelInfo().numSixDofContacts<<std::endl;  
  
  // float f = 0.0;
  // visualization_msgs::Marker points, line_strip, line_list;
  //   points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
  //   points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  //   points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  //   points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  //   points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  //   points.id = 0;
  //   line_strip.id = 1;
  //   line_list.id = 2;

  //   points.type = visualization_msgs::Marker::POINTS;
  //   line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  //   line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    // points.scale.x = 0.03;
    // points.scale.y = 0.03;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    // line_strip.scale.x = 0.05;
    // line_list.scale.x = 0.02;

    // Points are green
    // points.color.g = 1.0f;
    // points.color.a = 1.0;

    // Line strip is blue
    // line_strip.color.b = 1.0;
    // line_strip.color.a = 1.0;

    // Line list is red
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;

    // Create the vertices for the points and lines
      // geometry_msgs::Point p1,p2,p3,p4,p5,p6;
      // p1.x = positions[0][0];
      // p1.y = positions[0][1];
      // p1.z = positions[0][2];

      // p2.x = positions[1][0];
      // p2.y = positions[1][1];
      // p2.z = positions[1][2];

      // p3.x = positions[2][0];
      // p3.y = positions[2][1];
      // p3.z = positions[2][2];

      // p4.x = positions[3][0];
      // p4.y = positions[3][1];
      // p4.z = positions[3][2];

      // p5.x = com_P[1];
      // p5.y = com_P[2];
      // p5.z = com_P[3];

      // p6.x = pose[1];
      // p6.y = pose[2];
      // p6.z = pose[3];

      // points.points.push_back(p1);
      // points.points.push_back(p2);
      // points.points.push_back(p3);
      // points.points.push_back(p4);
      // points.points.push_back(p5);
      // points.points.push_back(p6);

      // line_list.points.push_back(p1);
      // line_list.points.push_back(p5);
      // line_list.points.push_back(p2);
      // line_list.points.push_back(p5);
      // line_list.points.push_back(p3);
      // line_list.points.push_back(p5);
      // line_list.points.push_back(p4);
      // line_list.points.push_back(p5);
      // line_list.points.push_back(p6);
      // line_list.points.push_back(p5);
      // line_strip.points.push_back(p);
      
      // The line list needs two points for each line
      // line_list.points.push_back(p);
      // p.z += 1.0;
      // line_list.points.push_back(p);

    // marker_pub.publish(points);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);
  // }
  // Visualization


  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    // contactFlag[i] = contactHandles_[i].isContact();
    contactFlag[i] = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getContactFlags(currentObservation_.time)[i];
    // contactFlag[i] = 0;
    }
    
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
