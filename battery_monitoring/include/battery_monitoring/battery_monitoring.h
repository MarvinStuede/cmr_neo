/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   battery_monitoring.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   01/2021
*
* @brief  Battery monitoring node which publishes a message if a recharge is needed
*/

#ifndef BATTERY_MONITORING_BATTERY_MONITORING_H
#define BATTERY_MONITORING_BATTERY_MONITORING_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <sensor_msgs/BatteryState.h>
#include <neo_msgs/RelayBoardV2.h>
#include <cmr_msgs/BatteryMonitoring.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <string>
#include <iostream>
#include <ctime>

class BatteryMonitoring
{
public:
    BatteryMonitoring(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber battery_state_subscriber_;
    ros::Subscriber state_subscriber_;
    ros::Publisher battery_monitoring_publisher_;
    ros::Timer my_timer_;

    // parameters
    double voltage_full;
    double voltage_start_recharging;
    double recharge_time;
    double recharge_time_night;

    bool charging;
    bool still_electrical_contact;

    bool full;
    bool recharge_needed;
    bool print_status;

    double sum_voltage_charging;
    int counter_charging;
    double sum_voltage_discharging;
    int counter_discharging;

    ros::Time start_time;
    ros::Duration timeout;
    ros::Duration timeout_night;
    int start_hour;
    bool night;

    sensor_msgs::BatteryState::ConstPtr battery_charge_state;
    neo_msgs::RelayBoardV2::ConstPtr base_state;
    cmr_msgs::BatteryMonitoring output_battery_charge_state;

    // callbacks
    void subscriberCallback(const sensor_msgs::BatteryStateConstPtr& battery_state);
    void statesubscriberCallback(const neo_msgs::RelayBoardV2ConstPtr& state);
    void timerCallback(const ros::TimerEvent &evt);

    //functions
    void resetCounters();
};

#endif // BATTERY_MONITORING_BATTERY_MONITORING_H
