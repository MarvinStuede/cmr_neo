
#include "cmr_neo_ultrasonic/us_avoid_collision.h"
#include <stdexcept>
#include <algorithm>

//########## CONSTRUCTOR ###############################################################################################
USCollision::USCollision(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
  //Ultrasonic subscribers
  sub_us_fl_ = node_->subscribe("/base/relayboard_v2/usboard/sensor1", 10, &USCollision::usFlCallback, this);
  sub_us_fr_ = node_->subscribe("/base/relayboard_v2/usboard/sensor2", 10, &USCollision::usFrCallback, this);
  sub_us_br_ = node_->subscribe("/base/relayboard_v2/usboard/sensor3", 10, &USCollision::usBrCallback, this);
  sub_us_bc_ = node_->subscribe("/base/relayboard_v2/usboard/sensor4", 10, &USCollision::usBcCallback, this);
  sub_us_bl_ = node_->subscribe("/base/relayboard_v2/usboard/sensor5", 10, &USCollision::usBlCallback, this);

  //Command velocity subscriber
  sub_cmd_vel_ = node_->subscribe("/cmd_vel", 10, &USCollision::cmdVelCallback, this);

  //Command velocity publisher
  pub_cmd_vel_ = node_->advertise<geometry_msgs::Twist>("/cmd_vel_us", 10,true);

  //Publisher
  pub_blocked_side_ = node_->advertise<cmr_msgs::blockedDirectionTime>("/blocked_side", 10,true);

  //Service to enable/disable sensors
  serv_activate_ = node_->advertiseService("sensor_activation",&USCollision::activateServiceCallback,this);

  //Timer to calculate limits and stop robot if command velocity is too old
  timer_ = node_->createTimer(ros::Duration(0.1), &USCollision::timerCallback, this);

  //Put sensors in vector
  sensors_.push_back(USSensor(MaxVelFactor(1,0,1,0))); //Sensor front left
  sensors_.push_back(USSensor(MaxVelFactor(1,0,0,1))); //Sensor front right
  sensors_.push_back(USSensor(MaxVelFactor(0,1,1,0))); //Sensor back right
  sensors_.push_back(USSensor(MaxVelFactor(0,1,1,1))); //Sensor back center
  sensors_.push_back(USSensor(MaxVelFactor(0,1,0,1))); //Sensor back left

  //Set limits to reduce speed and stop
  double thresh_reduce_front_left, thresh_stop_front_left;
  double thresh_reduce_front_right, thresh_stop_front_right;
  double thresh_reduce_back_right, thresh_stop_back_right;
  double thresh_reduce_back_center, thresh_stop_back_center;
  double thresh_reduce_back_left, thresh_stop_back_left;
  node_->param("thresh_reduce_front_left",thresh_reduce_front_left,0.5);
  node_->param("thresh_stop_front_left",thresh_stop_front_left,0.25);
  node_->param("thresh_reduce_front_right",thresh_reduce_front_right,0.5);
  node_->param("thresh_stop_front_right",thresh_stop_front_right,0.25);
  node_->param("thresh_reduce_back_right",thresh_reduce_back_right,0.5);
  node_->param("thresh_stop_back_right",thresh_stop_back_right,0.25);
  node_->param("thresh_reduce_back_center",thresh_reduce_back_center,0.6);
  node_->param("thresh_stop_back_center",thresh_stop_back_center,0.35);
  node_->param("thresh_reduce_back_left",thresh_reduce_back_left,0.5);
  node_->param("thresh_stop_back_left",thresh_stop_back_left,0.25);
  //Get parameter for max duration to keep velocity after last command
  node_->param("max_dur_vel",max_dur_vel_,1.0);

  if(thresh_stop_front_left > thresh_reduce_front_left || thresh_stop_front_right > thresh_reduce_front_right ||
     thresh_stop_back_right > thresh_reduce_back_right || thresh_stop_back_center > thresh_reduce_back_center ||
     thresh_stop_back_left > thresh_reduce_back_left){
    ROS_ERROR("Velocity reduce threshold must be larger than stop threshold");
    throw std::invalid_argument( "thresh_stop > thresh_reduce" );
  }
  else if(thresh_stop_front_left < 0 || thresh_reduce_front_left < 0 || thresh_stop_front_right < 0 ||
          thresh_reduce_front_right < 0 || thresh_stop_back_center < 0 || thresh_reduce_back_center < 0 ||
          thresh_stop_back_right < 0 || thresh_reduce_back_right < 0 ||thresh_stop_back_left < 0 ||
          thresh_reduce_back_left < 0 ){
    ROS_ERROR("Velocity reduce thresholds must be positive");
    throw std::invalid_argument( "Received negative parameters" );
  }

  //Set thresholds

  sensors_[0].thresh_reduce = thresh_reduce_front_left;
  sensors_[0].thresh_stop = thresh_stop_front_left;
  sensors_[1].thresh_reduce = thresh_reduce_front_right;
  sensors_[1].thresh_stop = thresh_stop_front_right;
  sensors_[2].thresh_reduce = thresh_reduce_back_right;
  sensors_[2].thresh_stop = thresh_stop_back_right;
  sensors_[3].thresh_reduce = thresh_reduce_back_center;
  sensors_[3].thresh_stop = thresh_stop_back_center;
  sensors_[4].thresh_reduce = thresh_reduce_back_left;
  sensors_[4].thresh_stop = thresh_stop_back_left;

  resetBlockedVariables();
}


MaxVelFactor USCollision::getMaxVelocities()
{
  MaxVelFactor velocities(1,1,1,1);
  std::vector<MaxVelFactor> resulting_max_vels;

  for(auto& sensor : sensors_){
    resulting_max_vels.push_back(sensor.getMaxAllowedVelocity());
  }

  return MaxVelFactor::mergeObjects(resulting_max_vels);

}

void USCollision::updateCmdVel()
{
  geometry_msgs::Twist cmd_vel_cmd = cmd_vel_recv_;

  //Calculate the "safe" velociy based on max_vels_ variable
  if(cmd_vel_recv_.linear.x < 0)
    cmd_vel_cmd.linear.x = std::max((float)cmd_vel_recv_.linear.x,(float)(cmd_vel_recv_.linear.x * max_vels_.getLimits().negX));

  if(cmd_vel_recv_.linear.x > 0)
    cmd_vel_cmd.linear.x = std::min((float)cmd_vel_recv_.linear.x,(float)(cmd_vel_recv_.linear.x * max_vels_.getLimits().posX));

  if(cmd_vel_recv_.angular.z < 0)
    cmd_vel_cmd.angular.z = std::max((float)cmd_vel_recv_.angular.z,(float)(cmd_vel_recv_.angular.z * max_vels_.getLimits().negAngular));

  if(cmd_vel_recv_.angular.z > 0)
    cmd_vel_cmd.angular.z = std::min((float)cmd_vel_recv_.angular.z,(float)(cmd_vel_recv_.angular.z * max_vels_.getLimits().posAngular));
  // Is cmd_vel_cmd nearly zero and different from cmd_vel_recv_? Yes means, there is an obstacle detected by the us-sensors.
  if((fabs(cmd_vel_recv_.angular.z) > 0.02) && (fabs(cmd_vel_cmd.angular.z) < 0.01))
  {
    if(first_time_blocked)
    {
      time_start_being_blocked = ros::Time::now();
      first_time_blocked = false;
    }
    // get the time for being blocked
    time_blocked = (ros::Time::now() - time_start_being_blocked).toSec();
    //bl > br
    if(sensors_[4].current_dist > sensors_[2].current_dist)
      right_side_blocked = true;
    else if(sensors_[4].current_dist < sensors_[2].current_dist)
      left_side_blocked = true;
  }else
  {
      resetBlockedVariables();
  }

  blocked_side_.time_blocked = time_blocked;
  blocked_side_.left_side_blocked = left_side_blocked;
  blocked_side_.right_side_blocked = right_side_blocked;
  pub_blocked_side_.publish(blocked_side_);
  pub_cmd_vel_.publish(cmd_vel_cmd);
  return;
}



bool USCollision::activateServiceCallback(cmr_msgs::setUSCollisionRequest &req, cmr_msgs::setUSCollisionResponse &res)
{
  if(req.id > sensors_.size()){
    ROS_ERROR("Sensor ID %d not found, max ID is %d",req.id,sensors_.size());
    res.success = false;
    return false;
  }

  if (req.id == 0){
    ROS_INFO("%s all sensors",req.enable ? "Enable" : "Disable");
    for(auto & sensor : sensors_){
      sensor.enabled = req.enable;
    }
    res.success = true;
    return true;
  }
  sensors_[req.id - 1].enabled = req.enable;
  ROS_INFO("%s sensor with ID %d",req.enable ? "Enable" : "Disable", req.id);

  res.success = true;
  return true;
}

void USCollision::timerCallback(const ros::TimerEvent &evt)
{
  max_vels_ = getMaxVelocities();
  float dur_last_cmd = (ros::Time::now() - tim_cmd_vel_).toSec();

  //Stop robot if last message too old
  if(dur_last_cmd > max_dur_vel_){
    cmd_vel_recv_.linear.x = 0;
    cmd_vel_recv_.angular.z = 0;
    updateCmdVel();
  }
}

void USCollision::cmdVelCallback(const geometry_msgs::Twist &twist)
{
  cmd_vel_recv_ = twist;
  tim_cmd_vel_ = ros::Time::now();
  updateCmdVel();
}

void USCollision::resetBlockedVariables()
{
  first_time_blocked = true;
  right_side_blocked = false;
  left_side_blocked = false;
  time_blocked = 0;
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "us_avoid_collision");

  ros::NodeHandle node_handle("~");
  USCollision us_collision(node_handle);

  ROS_INFO("Sonar collision avoidance started");
  ros::spin();

  return 0;
}
