/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * \file
 * \author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * \date   February 2019
 *
 * \brief  Node to stop robot based on ultrasonic sensor measurements
 */


#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "cmr_neo_ultrasonic/collision_dist.h"
#include "cmr_msgs/blockedDirectionTime.h"
#include "cmr_msgs/setUSCollision.h"

/**
 * @brief Struct for represent a US Sensor
 */
struct USSensor{
  USSensor() { }

  USSensor(MaxVelFactor influences){

    this->influences_ = influences;

  }
  /**
   * @brief Get allowed velocity in x/theta direction based on current measured distance
   * @return MaxVelFactor object with factor between 0 and 1 in each direction
   */
  MaxVelFactor getMaxAllowedVelocity(){
    VelLimits limits;
    MaxVelFactor factor(1,1,1,1);
    if(!enabled) return factor;

    limits.negX = limit(std::max(getVelFactor(),influences_.getLimits().negX));
    limits.posX = limit(std::max(getVelFactor(),influences_.getLimits().posX));
    limits.negAngular = limit(std::max(getVelFactor(),influences_.getLimits().negAngular));
    limits.posAngular = limit(std::max(getVelFactor(),influences_.getLimits().posAngular));

    factor.setLimits(limits);

    return factor;
  }
  /**
   * @brief Returns a velocity factor
   * @details Returns a linear interpolated factor based on the thresholds
   * @return Factor between 0 and 1
   */
  double getVelFactor(){
    if (current_dist > thresh_reduce) return 1.;
    if (current_dist <= thresh_stop) return 0.;
    return (current_dist - thresh_stop)/(thresh_reduce - thresh_stop); //Linear interpolation
  }

  //Limit value to interval between 0 and 1
  double limit(double val){return (val < 0 ? 0 : (val > 1 ? 1 : val));}

  //Describes which velocity directions should be blocked, in the case of a collision detection (currentDist below threshold)
  //0: should be blocked, 1: should not be blocked. E.g. if a sensor is mounted at the front, it would block positive x movement (1, 0, ... )
  MaxVelFactor influences_;

  double thresh_reduce =std::numeric_limits<double>::max();
  double thresh_stop =std::numeric_limits<double>::max();
  double current_dist = 0.;
  bool enabled = true;
};
/**
 * @brief USCollision class to implement the node
 */
class USCollision
{
public:
  USCollision(ros::NodeHandle &node_handle);

private:
  // node handle
  ros::NodeHandle *node_;

  //Subscribers for US measurements
  ros::Subscriber sub_us_fl_;
  ros::Subscriber sub_us_fr_;
  ros::Subscriber sub_us_br_;
  ros::Subscriber sub_us_bc_;
  ros::Subscriber sub_us_bl_;

  ros::ServiceServer serv_activate_;

  ros::Timer timer_;

  //Subscriber for velocity
  ros::Subscriber sub_cmd_vel_;

  ros::Publisher pub_cmd_vel_;

  // variables and publishers for the being blocked detection
  ros::Time time_start_being_blocked;
  bool first_time_blocked;
  bool left_side_blocked, right_side_blocked;
  int time_blocked;
  cmr_msgs::blockedDirectionTime blocked_side_;
  ros::Publisher pub_blocked_side_;
  void resetBlockedVariables();

  double max_dur_vel_ = 1.0;
  geometry_msgs::Twist cmd_vel_recv_;
  ros::Time tim_cmd_vel_;
  std::vector<USSensor> sensors_;
  MaxVelFactor max_vels_;

  //Callbacks for US Measurements
  void usFlCallback(const sensor_msgs::Range &range){sensors_[0].current_dist = range.range;}
  void usFrCallback(const sensor_msgs::Range &range){sensors_[1].current_dist = range.range;}
  void usBrCallback(const sensor_msgs::Range &range){sensors_[2].current_dist = range.range;}
  void usBcCallback(const sensor_msgs::Range &range){sensors_[3].current_dist = range.range;}
  void usBlCallback(const sensor_msgs::Range &range){sensors_[4].current_dist = range.range;}

  MaxVelFactor getMaxVelocities();
  void updateCmdVel();

  //Callbacks
  void cmdVelCallback(const geometry_msgs::Twist &twist);
  bool activateServiceCallback(cmr_msgs::setUSCollisionRequest &req, cmr_msgs::setUSCollisionResponse &res);
  void timerCallback(const ros::TimerEvent &evt);


};
