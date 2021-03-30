// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-05-28
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration_consumer.h>

#if __has_include(<filesystem>)
#  include <filesystem>
   namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
# include <experimental/filesystem>
  namespace fs = std::experimental::filesystem;
#elif __has_include(<boost/filesystem.hpp>)
# include <boost/filesystem.hpp>
  namespace fs = boost::filesystem;
#endif

namespace ur_calibration
{
CalibrationConsumer::CalibrationConsumer(std::string dh_config_filename) : calibrated_(false), dh_config_filename_(dh_config_filename)
{
}

bool CalibrationConsumer::consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product)
{
  auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
  if (kin_info != nullptr)
  {
    ROS_INFO("%s", product->toString().c_str());
    DHRobot my_robot;
    if (fs::exists(dh_config_filename_))
    {
      YAML::Node dh_config_file = YAML::LoadFile(dh_config_filename_);
      try {
        for (int i = 1; i <= 6; i++){
          std::string joint_name = "joint_" + std::to_string(i);
          my_robot.segments_.push_back(
            DHSegment(dh_config_file["dh_parameter"][joint_name]["d"].as<double>(),
                      dh_config_file["dh_parameter"][joint_name]["a"].as<double>(),
                      dh_config_file["dh_parameter"][joint_name]["theta"].as<double>(),
                      dh_config_file["dh_parameter"][joint_name]["alpha"].as<double>()));
        }
      }
      catch (YAML::TypedBadConversion<double> ye)
      {
        ROS_ERROR_STREAM("Could not load yaml file correctly.");
        ros::shutdown();
      }
    }
    else
    {
      ROS_INFO_STREAM("Loading dh values from UR controller");
      ROS_INFO("%s", product->toString().c_str());
      for (size_t i = 0; i < kin_info->dh_a_.size(); ++i)
      {
        my_robot.segments_.push_back(
            DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
      }
    }

    Calibration calibration(my_robot);
    calibration.correctChain();

    calibration_parameters_ = calibration.toYaml();
    calibration_parameters_["kinematics"]["hash"] = kin_info->toHash();
    calibrated_ = true;
  }
  return true;
}

YAML::Node CalibrationConsumer::getCalibrationParameters() const
{
  if (!calibrated_)
  {
    throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
  }
  return calibration_parameters_;
}
}  // namespace ur_calibration
