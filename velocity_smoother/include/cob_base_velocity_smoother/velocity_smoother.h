/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <velocity_smoother/paramsConfig.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <geometry_msgs/PolygonStamped.h>
#include <spencer_tracking_msgs/TrackedPersons.h>




/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_base_velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother
{
public:
  VelocitySmoother(const std::string &name);

  ~VelocitySmoother()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

  bool init(ros::NodeHandle& nh);
  void spin();
  void shutdown() { shutdown_req = true; };

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback;  /**< What source to use as robot velocity feedback */

  std::string name;
  double speed_lim_vx, accel_lim_vx, decel_lim_vx, decel_lim_vx_safe;
  double speed_lim_vy, accel_lim_vy, decel_lim_vy, decel_lim_vy_safe;
  double speed_lim_w, accel_lim_w, decel_lim_w, decel_lim_w_safe;
  double decel_factor, decel_factor_safe;

  //***** laserscan params **********
  double speed_laser_lim_x=1.0;
  double slow_distance_x=2.0;
  double stop_distance_x=0.2;

  double speed_laser_lim_z=1.0;
  double slow_distance_z=2.0;
  double stop_distance_z=0.2;

  double laser_min;
  ros::Time laser_cb_time=ros::Time::now();

  double slow_distance_faktor_laser=1.0;
  double stop_distance_faktor_laser=1.0;

  //***** veloscan params **********
  ros::Time velodyne_laser_cb_time=ros::Time::now();
  double velodyne_laser_front_min;
  double velodyne_laser_back_min;
  double velodyne_laser_rot_min;

  double speed_velo_lim_x=1.0;


  double slow_distance_velo_z=2.0;
  double stop_distance_velo_z=0.2;
  double speed_velo_lim_z=1;



  double speed_velo_back_lim_x;
  double speed_velo_front_lim_x;
  double speed_velo_rot_right;
  double speed_velo_rot_left;


  bool _enable_laser;
  bool _enable_velo;

  double slow_distance_velo_x_front=1.3;
  double stop_distance_velo_x_front=0.7;
  double angle_left_x_front=-2.356;
  double angle_right_x_front=2.356;

  double slow_distance_velo_x_back=1.0;
  double stop_distance_velo_x_back=0.5;
  double angle_left_x_back=1.57;
  double angle_right_x_back=-1.57;

  double slow_distance_velo_rot_1=1.0;
  double stop_distance_velo_rot_1=0.5;
  double angle_left_rot_1=-1.9;
  double angle_right_rot_1=-3.0;

  double slow_distance_velo_rot_2=1.0;
  double stop_distance_velo_rot_2=0.5;
  double angle_left_rot_2=-0.3;
  double angle_right_rot_2=-1.2;

  double slow_distance_velo_rot_3=1.0;
  double stop_distance_velo_rot_3=0.5;
  double angle_left_rot_3=0.3;
  double angle_right_rot_3=1.2;

  double slow_distance_velo_rot_4=1.3;
  double stop_distance_velo_rot_4=0.7;
  double angle_left_rot_4=1.9;
  double angle_right_rot_4=3.0;

  int field=0;
  double field_thres=0.5;
  bool debug_fil=false;

  std::vector<geometry_msgs::Point> polygon_slow;
std::vector<geometry_msgs::Point> polygon_stop;

double speed_spencer_x=1.0;
ros::Time spencer_cb_time=ros::Time::now();
bool _enable_spencer;

  double frequency;

  std::string people_slow_polygon;
  std::string people_stop_polygon;

  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist  current_vel;
  geometry_msgs::Twist   target_vel;

  bool                 shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  bool                 input_active;
  double                cb_avg_time;
  ros::Time            last_cb_time;
  std::vector<double> period_record; /**< Historic of latest periods between velocity commands */
  unsigned int             pr_next; /**< Next position to fill in the periods record buffer */

  ros::Subscriber odometry_sub;    /**< Current velocity from odometry */
  ros::Subscriber current_vel_sub; /**< Current velocity from commands sent to the robot, not necessarily by this node */
  ros::Subscriber raw_in_vel_sub;  /**< Incoming raw velocity commands */
  ros::Subscriber laser_sub;
  ros::Subscriber velodyne_laser_sub;
  ros::Subscriber spencer_track_sub;
  ros::Publisher  smooth_vel_pub;  /**< Outgoing smoothed velocity commands */
  ros::Publisher slow_poly_pub;
  ros::Publisher stop_poly_pub;
  ros::Timer my_timer_;

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg);
  void veoldyne_laserCB(const sensor_msgs::LaserScan::ConstPtr& msg);
void personCB(const spencer_tracking_msgs::DetectedPersonsConstPtr &spencer_msg);
void personCB_online(const spencer_tracking_msgs::DetectedPersonsConstPtr &spencer_msg, const geometry_msgs::PolygonStampedConstPtr& msg_polygon);
void timerCallback(const ros::TimerEvent &evt);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig> *             dynamic_reconfigure_server;
  dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig>::CallbackType dynamic_reconfigure_callback;
  void reconfigCB(cob_base_velocity_smoother::paramsConfig &config, uint32_t unused_level);
};

} // cob_base_velocity_smoother

#endif /* VELOCITY_SMOOTHER_HPP_ */
