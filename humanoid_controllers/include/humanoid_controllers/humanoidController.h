//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <humanoid_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <humanoid_dummy/visualization/HumanoidVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <humanoid_estimation/StateEstimateBase.h>
#include <humanoid_interface/HumanoidInterface.h>
#include <humanoid_wbc/WbcBase.h>

#include "humanoid_controllers/SafetyChecker.h"
#include "humanoid_controllers/visualization/humanoidSelfCollisionVisualization.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"

namespace humanoid_controller{
using namespace ocs2;
using namespace humanoid;

class humanoidController : public controller_interface::Controller<HybridJointInterface>{
 public:
  humanoidController() = default;
  ~humanoidController() override;
  bool init(HybridJointInterface* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupHumanoidInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void jointStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  // Interface
  std::shared_ptr<HumanoidInterface> HumanoidInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<HumanoidVisualizer> robotVisualizer_;
  ros::Publisher observationPublisher_;

  //Controller Interface
  ros::Publisher targetTorquePub_;
  ros::Publisher targetPosPub_;
    ros::Publisher targetVelPub_;
    ros::Publisher targetKpPub_;
    ros::Publisher targetKdPub_;
    ros::Subscriber jointPosVelSub_;
    ros::Subscriber imuSub_;

    // Node Handle
    ros::NodeHandle controllerNh_;


 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  size_t jointNum_ = 12;
  vector_t jointPos_, jointVel_;
  Eigen::Quaternion<scalar_t> quat_;
  contact_flag_t contactFlag_;
  vector3_t angularVel_, linearAccel_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
  size_t plannedMode_ = 3;
  vector_t defalutJointPos_;
};

class humanoidCheaterController : public humanoidController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace humanoid_controller

