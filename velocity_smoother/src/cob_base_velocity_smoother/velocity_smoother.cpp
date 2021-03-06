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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <cob_base_velocity_smoother/velocity_smoother.h>

#include <boost/thread.hpp>
#include <footprint.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/



namespace cob_base_velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const std::string &name)
: name(name)
, shutdown_req(false)
, input_active(false)
, pr_next(0)
, dynamic_reconfigure_server(NULL)
{
}


void VelocitySmoother::reconfigCB(cob_base_velocity_smoother::paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f",
           config.speed_lim_vx, config.speed_lim_vy, config.speed_lim_w, config.accel_lim_vx, config.accel_lim_vy, config.accel_lim_w, config.decel_factor, config.decel_factor_safe);

  speed_lim_vx  = config.speed_lim_vx;
  speed_lim_vy  = config.speed_lim_vy;
  speed_lim_w  = config.speed_lim_w;
  accel_lim_vx  = config.accel_lim_vx;
  accel_lim_vy  = config.accel_lim_vy;
  accel_lim_w  = config.accel_lim_w;
  decel_factor = config.decel_factor;
  decel_factor_safe = config.decel_factor_safe;
  decel_lim_vx  = decel_factor*accel_lim_vx;
  decel_lim_vy  = decel_factor*accel_lim_vy;
  decel_lim_w  = decel_factor*accel_lim_w;
  decel_lim_vx_safe = decel_factor_safe*accel_lim_vx;
  decel_lim_vy_safe = decel_factor_safe*accel_lim_vy;
  decel_lim_w_safe = decel_factor_safe*accel_lim_w;

  _enable_laser=config.enable_laser;
  _enable_velo=config.enable_velo;

   slow_distance_velo_x_front=config.slow_distance_velo_x_front;
   stop_distance_velo_x_front=config.stop_distance_velo_x_front;
   angle_left_x_front=config.angle_left_x_front*3.141/180.0;
   angle_right_x_front=config.angle_right_x_front*3.141/180.0;

   slow_distance_velo_x_back=config.slow_distance_velo_x_back;
   stop_distance_velo_x_back=config.stop_distance_velo_x_back;
   angle_left_x_back=config.angle_left_x_back*3.141/180.0;
   angle_right_x_back=config.angle_right_x_back*3.141/180.0;

   slow_distance_velo_rot_1=config.slow_distance_velo_rot_1;
   stop_distance_velo_rot_1=config.stop_distance_velo_rot_1;
   angle_left_rot_1=config.angle_left_rot_1*3.141/180.0;
   angle_right_rot_1=config.angle_right_rot_1*3.141/180.0;

   slow_distance_velo_rot_2=config.slow_distance_velo_rot_2;
   stop_distance_velo_rot_2=config.stop_distance_velo_rot_2;
   angle_left_rot_2=config.angle_left_rot_2*3.141/180.0;
   angle_right_rot_2=config.angle_right_rot_2*3.141/180.0;

   slow_distance_velo_rot_3=config.slow_distance_velo_rot_3;
   stop_distance_velo_rot_3=config.stop_distance_velo_rot_3;
   angle_left_rot_3=config.angle_left_rot_3*3.141/180.0;
   angle_right_rot_3=config.angle_right_rot_3*3.141/180.0;

   slow_distance_velo_rot_4=config.slow_distance_velo_rot_4;
   stop_distance_velo_rot_4=config.stop_distance_velo_rot_4;
   angle_left_rot_4=config.angle_left_rot_4*3.141/180.0;
   angle_right_rot_4=config.angle_right_rot_4*3.141/180.0;

   field=config.filter_field;
   field_thres=config.filter_thres;
   debug_fil=config.debug_filter;
   _enable_spencer=config.enable_spencer;






}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
     double lim_faktor_laser_x;
     double lim_faktor_velo_x_front, lim_faktor_velo_x_back,lim_faktor_velo_z_rot_left, lim_faktor_velo_z_rot_right;
     double lim_faktor_spencer_x;

    if((ros::Time::now()-laser_cb_time).toSec()<1.0)
    {
       lim_faktor_laser_x=speed_laser_lim_x;
    }
    else {
        lim_faktor_laser_x=1.0;
       // speed_laser_lim_z=1.0;
    }

    if((ros::Time::now()-velodyne_laser_cb_time).toSec()<1.0)
    {
       lim_faktor_velo_x_front=speed_velo_front_lim_x;
       lim_faktor_velo_x_back=speed_velo_back_lim_x;
       lim_faktor_velo_z_rot_left=speed_velo_rot_left;
       lim_faktor_velo_z_rot_right=speed_velo_rot_right;
    }
    else {
        lim_faktor_velo_x_front=1.0;
        lim_faktor_velo_x_back=1.0;
        lim_faktor_velo_z_rot_left=1.0;
        lim_faktor_velo_z_rot_right=1.0;
    }



    if((ros::Time::now()-laser_cb_time).toSec()<1.0)
    {
       lim_faktor_spencer_x=speed_spencer_x;
    }
    else {
        lim_faktor_spencer_x=1.0;

    }


  double lim_faktor_x_front=std::min(lim_faktor_laser_x,lim_faktor_velo_x_front);


  //ROS_INFO("lim_faktor_x_front: %f",lim_faktor_x_front);
  //ROS_INFO("lim_faktor_velo_x_back: %f",lim_faktor_velo_x_back);
//  ROS_INFO("lim_faktor_velo_z_rot_left: %f",lim_faktor_velo_z_rot_left);
//  ROS_INFO("lim_faktor_velo_z_rot_right: %f",lim_faktor_velo_z_rot_right);

  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  }
  else
  {
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
    cb_avg_time = 0.1;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // Bound speed with the maximum values
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_vx*lim_faktor_x_front) : std::max(msg->linear.x,  -speed_lim_vx*lim_faktor_velo_x_back);
  target_vel.linear.y  =
      msg->linear.y  > 0.0 ? std::min(msg->linear.y,  speed_lim_vy) : std::max(msg->linear.y,  -speed_lim_vy);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w*lim_faktor_velo_z_rot_left) : std::max(msg->angular.z, -speed_lim_w*lim_faktor_velo_z_rot_right);
  //std::cout<<"target_vel.linear.x "<<target_vel.linear.x <<std::endl;
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (robot_feedback == ODOMETRY)
    current_vel = msg->twist.twist;

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (robot_feedback == COMMANDS)
    current_vel = *msg;

  // ignore otherwise
}

void VelocitySmoother::laserCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   if(_enable_laser)
   {
   laser_cb_time=ros::Time::now();
    int detection_angle=int((msg->angle_max-1.5)/msg->angle_increment);;

    std::vector<double> laser_range(std::begin(msg->ranges)+detection_angle,std::end(msg->ranges)-detection_angle);
    std::vector<double> slow_factors;

    for (auto it=laser_range.begin();it!=laser_range.end();it++) {

        if(*it==0)
        {
            *it=100.0;
        }
        double range=*it;
        double angle=((std::distance(laser_range.begin(),it))+detection_angle)*msg->angle_increment+msg->angle_min;
        double stop_distance =(0.3071 -0.1057*cos(2.013*angle))*stop_distance_faktor_laser;
        double slow_distance =(-0.4053*pow(angle,2)+2)*slow_distance_faktor_laser;

        double m ,b;
        m=1/(slow_distance-stop_distance);
        b=1-m*slow_distance;
        double temp_speed_laser_lim=m*range+b;
        slow_factors.push_back(temp_speed_laser_lim);
        //std::cout<<"range: "<<range<<"  angle: "<<angle<<"   temp_speed_laser_lim: "<<temp_speed_laser_lim<<std::endl;

    }



//    for(auto it=laser_range.begin();it!=laser_range.end();)
//    {


//        if(*it==0)
//        {
//            //std::cout<<"hallo . "<<*it<<std::endl;
//            it=laser_range.erase(it);
//        }
//        else {
//            it++;
//        }
//    }

   /* for(auto range:laser_range)
    {
        std::cout<<range<<std::endl;
    }
    */

   double min_laser_factors =*std::min_element(slow_factors.begin(),slow_factors.end());


    speed_laser_lim_x=min_laser_factors  > 0.0 ? std::min(min_laser_factors,  1.0) : std::max(min_laser_factors,  0.0);
//std::cout<<"speed_laser_lim_x"<<speed_laser_lim_x<<std::endl;

//    m=1/(slow_distance_z-stop_distance_z);
//    b=1-m*slow_distance_z;
//    double temp_speed_laser_lim_z=m*temp_laser_min+b;
   //double temp_speed_laser_lim_z= 5.0*std::pow((laser_min-0.5),3)+0.5;

   // speed_laser_lim_z=temp_speed_laser_lim_z  > 0.0 ? std::min(temp_speed_laser_lim_z,  1.0) : std::max(temp_speed_laser_lim_z,  0.0);

}
}

void VelocitySmoother::veoldyne_laserCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(_enable_velo)
    {
    velodyne_laser_cb_time=ros::Time::now();
     //int detection_angle=int(2.35/msg->angle_increment);
//int detection_angle=0;
     std::vector<double> laser_range(std::begin(msg->ranges),std::end(msg->ranges));

   std::vector<double> slow_factors_back;
   std::vector<double> slow_factors_front;
   std::vector<double> slow_factors_rot_left;
   std::vector<double> slow_factors_rot_right;
     for (auto it=laser_range.begin();it!=laser_range.end();it++) {

         if(*it==0)
         {
             *it=100.0  ;
         }

         double range=*it;
         double angle=(std::distance(laser_range.begin(),it))*msg->angle_increment+msg->angle_min;
        // std::cout<<"range: "<<range<<"angle: "<<angle<<std::endl;



         if(angle > angle_left_x_front  && angle <angle_right_x_front )
         {
              if(debug_fil)
                 {
           ROS_INFO_STREAM(angle_left_x_front<<" "<<angle_right_x_front);
             ROS_INFO_STREAM("angle: "<<angle);
               }
             double m ,b;
             m=1/(slow_distance_velo_x_front-stop_distance_velo_x_front);
             b=1-m*slow_distance_velo_x_front;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_front.push_back(temp_speed_laser_lim);
        }



          if((angle < angle_left_rot_1 && angle > angle_right_rot_1) )
         {
             double m ,b;
             m=1/(slow_distance_velo_rot_1-stop_distance_velo_rot_1);
             b=1-m*slow_distance_velo_rot_1;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_rot_left.push_back(temp_speed_laser_lim);
         }

          if( (angle >angle_left_rot_3 && angle <angle_right_rot_3) )
         {
             double m ,b;
             m=1/(slow_distance_velo_rot_3-stop_distance_velo_rot_3);
             b=1-m*slow_distance_velo_rot_3;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_rot_left.push_back(temp_speed_laser_lim);
         }


          if( (angle < angle_left_rot_2 && angle > angle_right_rot_2) )
         {

             double m ,b;
             m=1/(slow_distance_velo_rot_2-stop_distance_velo_rot_2);
             b=1-m*slow_distance_velo_rot_2;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_rot_right.push_back(temp_speed_laser_lim);
         }

          if((angle > angle_left_rot_4 && angle <angle_right_rot_4)   )
         {
             double m ,b;
             m=1/(slow_distance_velo_rot_4-stop_distance_velo_rot_4);
             b=1-m*slow_distance_velo_rot_4;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_rot_right.push_back(temp_speed_laser_lim);
         }



          if(angle < angle_left_x_back  &&  angle > angle_right_x_back)
         {

             double m ,b;
             m=1/(slow_distance_velo_x_back-stop_distance_velo_x_back);
             b=1-m*slow_distance_velo_x_back;
             double temp_speed_laser_lim=m*range+b;
             slow_factors_back.push_back(temp_speed_laser_lim);
         }

     }

    
     std::vector<double> slow_factors_back_filtered=footprint::middle_filter(slow_factors_back , field, field_thres);
    std::vector<double> slow_factors_front_filtered=footprint::middle_filter(slow_factors_front , field, field_thres);
    std::vector<double> slow_factors_rot_left_filtered=footprint::middle_filter(slow_factors_rot_left , field, field_thres);
   std::vector<double> slow_factors_rot_right_filtered=footprint::middle_filter(slow_factors_rot_right , field, field_thres);



     double min_factors_back =*std::min_element(slow_factors_back_filtered.begin(),slow_factors_back_filtered.end());
     double min_factors_back2 =*std::min_element(slow_factors_back.begin(),slow_factors_back.end());
     double min_factors_front =*std::min_element(slow_factors_front_filtered.begin(),slow_factors_front_filtered.end());
     double min_factors_rot_left =*std::min_element(slow_factors_rot_left_filtered.begin(),slow_factors_rot_left_filtered.end());
     double min_factors_rot_right =*std::min_element(slow_factors_rot_right_filtered.begin(),slow_factors_rot_right_filtered.end());

     if(debug_fil)
     ROS_INFO_STREAM("min_factors_back: "<<min_factors_back <<" size: "<< slow_factors_back_filtered.size()<<" min_factors_back2: "<<min_factors_back2<<" size :"<< slow_factors_back.size());

     speed_velo_back_lim_x=min_factors_back  > 0.0 ? std::min(min_factors_back,  1.0) : std::max(min_factors_back,  0.0);
    //ROS_INFO_STREAM("speed_velo_back_lim_x: "<<speed_velo_back_lim_x);
    speed_velo_front_lim_x=min_factors_front  > 0.0 ? std::min(min_factors_front,  1.0) : std::max(min_factors_front,  0.0);
    speed_velo_rot_right=min_factors_rot_right  > 0.0 ? std::min(min_factors_rot_right,  1.0) : std::max(min_factors_rot_right,  0.0);
    speed_velo_rot_left=min_factors_rot_left  > 0.0 ? std::min(min_factors_rot_left,  1.0) : std::max(min_factors_rot_left,  0.0);
//     for(auto it=laser_range.begin();it!=laser_range.end();)
//     {


//         if(*it==0)
//         {
//             //std::cout<<"hallo . "<<*it<<std::endl;
//             it=laser_range.erase(it);
//         }
//         else {
//             it++;
//         }
//     }

//     for(auto range:laser_range)
//     {
//         std::cout<<range<<std::endl;
//     }


//   auto laser_min_it =std::min_element(laser_range.begin(),laser_range.end());
//    laser_min=*laser_min_it;
//   double laser_min_angle=(std::distance(laser_range.begin(),laser_min_it))*msg->angle_increment+msg->angle_min;



    //std::cout<<"min_range: "<<laser_min<<"  angle: "<<laser_min_angle<<std::endl;
    }
}


void VelocitySmoother::personCB_online(const spencer_tracking_msgs::DetectedPersonsConstPtr &spencer_msg, const geometry_msgs::PolygonStampedConstPtr& msg_polygon)
{


//    std::vector<double> slow_factors_people;

//    for (auto person :spencer_msg->detections ) {
//        double x=person.pose.pose.position.x;
//        double y=person.pose.pose.position.y;

//        footprint::InPolygon_point()
//R

//    }





}

void VelocitySmoother::personCB(const spencer_tracking_msgs::DetectedPersonsConstPtr &spencer_msg)
{

    if(_enable_spencer)
    {

    spencer_cb_time=ros::Time::now();

    if(!spencer_msg->detections.empty())
    {
     std::vector<double> slow_factors_people;

     for (auto person:spencer_msg->detections) {

         int flag_slow=footprint::InPolygon_point(polygon_slow,person.pose.pose.position.x,person.pose.pose.position.y);
         int flag_stop=footprint::InPolygon_point(polygon_stop,person.pose.pose.position.x,person.pose.pose.position.y);

         if(flag_stop)
         {
             slow_factors_people.push_back(0.0);
        }
         if(flag_slow)
         {
             slow_factors_people.push_back(0.3);
         }


     }
     double min_factors;
     if(!slow_factors_people.empty())
       {
         min_factors =*std::min_element(slow_factors_people.begin(),slow_factors_people.end());
       }
      else
       { 
          min_factors=1.0;
       }
         speed_spencer_x=min_factors  > 0.0 ? std::min(min_factors,  1.0) : std::max(min_factors,  0.0);
    }
 else {
     speed_spencer_x=1.0;
      }
    //ROS_INFO_STREAM(speed_spencer_x);
   
    }

}

void VelocitySmoother::timerCallback(const ros::TimerEvent &evt)
{

    if(polygon_slow.size()>2)
    {
        geometry_msgs::PolygonStamped poly_temp;
        poly_temp.header.stamp=ros::Time::now();
        poly_temp.header.frame_id="base_link";
        
        for (auto point:polygon_slow) {
            geometry_msgs::Point32 temp_point;
            temp_point.x=point.x;
            temp_point.y=point.y;
            temp_point.z=0.0;
            poly_temp.polygon.points.push_back(temp_point);
        }
     slow_poly_pub.publish(poly_temp);
    }

    if(polygon_stop.size()>2)
    {
        geometry_msgs::PolygonStamped poly_temp;
        poly_temp.header.stamp=ros::Time::now();
        poly_temp.header.frame_id="base_link";
        
        for (auto point:polygon_stop) {
            geometry_msgs::Point32 temp_point;
            temp_point.x=point.x;
            temp_point.y=point.y;
            temp_point.z=0.0;
            poly_temp.polygon.points.push_back(temp_point);

        }
    stop_poly_pub.publish(poly_temp);
    }

}


void VelocitySmoother::spin()
{
  double period = 1.0/frequency;
  ros::Rate spin_rate(frequency);

  double decel_vx;
  double decel_vy;
  double decel_w;

  while (! shutdown_req && ros::ok())
  {

    if ((input_active == true) && (cb_avg_time > 0.0) &&
        ((ros::Time::now() - last_cb_time).toSec() > 0.5))
    {
//std::min(9.0*cb_avg_time, 0.5)
      // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
      // this, just in case something went wrong with our input, or he just forgot good manners...
      // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
      // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
      // several messages arrive with the same time and so lead to a zero median
      input_active = false;
      if (IS_ZERO_VEOCITY(target_vel) == false)
      {
        ROS_WARN_STREAM("Velocity Smoother : input got inactive leaving us a non-zero target velocity ("
              << target_vel.linear.x << ", " << target_vel.linear.y << ", " << target_vel.angular.z << "), zeroing...[" << name << "]");
        target_vel = ZERO_VEL_COMMAND;
      }
    }

    if(input_active)
    {
      decel_vx = decel_lim_vx;
      decel_vy = decel_lim_vy;
      decel_w = decel_lim_w;
    }
    else
    {
      //increase decel factor because this is a safty case, no more commands means we should stop as fast as it is safe
      decel_vx = decel_lim_vx_safe;
      decel_vy = decel_lim_vy_safe;
      decel_w = decel_lim_w_safe;
    }

    if ((robot_feedback != NONE) && (input_active == true) && (cb_avg_time > 0.0) &&
        (((ros::Time::now() - last_cb_time).toSec() > 0.5)     || // 5 missing msgs
          (std::abs(current_vel.linear.x  - last_cmd_vel.linear.x)  > 0.2) ||
          (std::abs(current_vel.linear.y  - last_cmd_vel.linear.y)  > 0.2) ||
          (std::abs(current_vel.angular.z - last_cmd_vel.angular.z) > 2.0)))
    {

      // If the publisher has been inactive for a while, or if our current commanding differs a lot
      // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
      // This can happen mainly due to preemption of current controller on velocity multiplexer.
      // TODO: current command/feedback difference thresholds are 진짜 arbitrary; they should somehow
      // be proportional to max v and w...
      // The one for angular velocity is very big because is it's less necessary (for example the
      // reactive controller will never make the robot spin) and because the gyro has a 15 ms delay
      ROS_WARN("Using robot velocity feedback (%s) instead of last command: %f, %f, %f, %f",
                robot_feedback == ODOMETRY ? "odometry" : "end commands",
               (ros::Time::now()      - last_cb_time).toSec(),
                current_vel.linear.x  - last_cmd_vel.linear.x,
                current_vel.linear.y  - last_cmd_vel.linear.y,
                current_vel.angular.z - last_cmd_vel.angular.z);
      last_cmd_vel = current_vel;
    }

    geometry_msgs::TwistPtr cmd_vel;

    if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
        (target_vel.linear.y  != last_cmd_vel.linear.y) ||
        (target_vel.angular.z != last_cmd_vel.angular.z))
    {
      // Try to reach target velocity ensuring that we don't exceed the acceleration limits
      cmd_vel.reset(new geometry_msgs::Twist(target_vel));

      double vx_inc, vy_inc, w_inc, max_vx_inc, max_vy_inc, max_w_inc;

      vx_inc = target_vel.linear.x - last_cmd_vel.linear.x;
      if ((robot_feedback == ODOMETRY) && (current_vel.linear.x*target_vel.linear.x < 0.0))
      {
        // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
        max_vx_inc = decel_vx*period;
      }
      else
      {
        max_vx_inc = ((vx_inc*target_vel.linear.x > 0.0)?accel_lim_vx:decel_vx)*period;
      }

      vy_inc = target_vel.linear.y - last_cmd_vel.linear.y;
      if ((robot_feedback == ODOMETRY) && (current_vel.linear.y*target_vel.linear.y < 0.0))
      {
        // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
        max_vy_inc = decel_vy*period;
      }
      else
      {
        max_vy_inc = ((vy_inc*target_vel.linear.y > 0.0)?accel_lim_vy:decel_vy)*period;
      }

      w_inc = target_vel.angular.z - last_cmd_vel.angular.z;
      if ((robot_feedback == ODOMETRY) && (current_vel.angular.z*target_vel.angular.z < 0.0))
      {
        // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
        max_w_inc = decel_w*period;
      }
      else
      {
        max_w_inc = ((w_inc*target_vel.angular.z > 0.0)?accel_lim_w:decel_w)*period;
      }

/*
      // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
      // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
      // which velocity (v or w) must be overconstrained to keep the direction provided as command
      double MA = sqrt(    vx_inc *     vx_inc +     w_inc *     w_inc);
      double MB = sqrt(max_vx_inc * max_vx_inc + max_w_inc * max_w_inc);

      double Av = std::abs(vx_inc) / MA;
      double Aw = std::abs(w_inc) / MA;
      double Bv = max_vx_inc / MB;
      double Bw = max_w_inc / MB;
      double theta = atan2(Bw, Bv) - atan2(Aw, Av);

      if (theta < 0)
      {
        // overconstrain linear velocity
        max_vx_inc = (max_w_inc*std::abs(vx_inc))/std::abs(w_inc);
      }
      else
      {
        // overconstrain angular velocity
        max_w_inc = (max_vx_inc*std::abs(w_inc))/std::abs(vx_inc);
      }
*/
      if (std::abs(vx_inc) > max_vx_inc)
      {
        // we must limit linear velocity
        cmd_vel->linear.x  = last_cmd_vel.linear.x  + sign(vx_inc)*max_vx_inc;
      }

      if (std::abs(vy_inc) > max_vy_inc)
      {
        // we must limit linear velocity
        cmd_vel->linear.y  = last_cmd_vel.linear.y  + sign(vy_inc)*max_vy_inc;
      }

      if (std::abs(w_inc) > max_w_inc)
      {
        // we must limit angular velocity
        cmd_vel->angular.z = last_cmd_vel.angular.z + sign(w_inc)*max_w_inc;
      }

      smooth_vel_pub.publish(cmd_vel);
      last_cmd_vel = *cmd_vel;
    }
    else if (input_active == true)
    {
      // We already reached target velocity; just keep resending last command while input is active
      cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
      smooth_vel_pub.publish(cmd_vel);
    }

    spin_rate.sleep();
  }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

 polygon_slow=footprint::makeFootprintFromParams(nh,"people_slow_polygon");
polygon_stop=footprint::makeFootprintFromParams(nh,"people_stop_polygon");


  // Optional parameters
  int feedback;
  nh.param("frequency",      frequency,     20.0);
  nh.param("decel_factor",   decel_factor,   1.0);
  nh.param("decel_factor_safe",   decel_factor_safe,   1.0);
  nh.param("robot_feedback", feedback, (int)NONE);

  if ((int(feedback) < NONE) || (int(feedback) > COMMANDS))
  {
    ROS_WARN("Invalid robot feedback type (%d). Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)",
             feedback);
    feedback = NONE;
  }

  robot_feedback = static_cast<RobotFeedbackType>(feedback);

  // Mandatory parameters
  if ((nh.getParam("speed_lim_vx", speed_lim_vx) == false) ||
      (nh.getParam("speed_lim_vy", speed_lim_vy) == false) ||
      (nh.getParam("speed_lim_w", speed_lim_w) == false))
  {
    ROS_ERROR("Missing velocity limit parameter(s)");
    return false;
  }

  if ((nh.getParam("accel_lim_vx", accel_lim_vx) == false) ||
      (nh.getParam("accel_lim_vy", accel_lim_vy) == false) ||
      (nh.getParam("accel_lim_w", accel_lim_w) == false))
  {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  // Deceleration can be more aggressive, if necessary
  decel_lim_vx = decel_factor*accel_lim_vx;
  decel_lim_vy = decel_factor*accel_lim_vy;
  decel_lim_w = decel_factor*accel_lim_w;
  // In safety cases (no topic command anymore), deceleration should be very aggressive
  decel_lim_vx_safe = decel_factor_safe*accel_lim_vx;
  decel_lim_vy_safe = decel_factor_safe*accel_lim_vy;
  decel_lim_w_safe = decel_factor_safe*accel_lim_w;

  // Publishers and subscribers
  odometry_sub    = nh.subscribe("/odom",      1, &VelocitySmoother::odometryCB, this);
  current_vel_sub = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
  raw_in_vel_sub  = nh.subscribe("/cmd_vel_us",   1, &VelocitySmoother::velocityCB, this);
  laser_sub = nh.subscribe("/base/laser/scan_filtered",1, &VelocitySmoother::laserCB,this );
  velodyne_laser_sub=nh.subscribe("/velodyne_laserscan",1,&VelocitySmoother::veoldyne_laserCB,this);
  spencer_track_sub=nh.subscribe("/spencer/perception/detected_persons",1,&VelocitySmoother::personCB,this);
  smooth_vel_pub  = nh.advertise <geometry_msgs::Twist> ("/cmd_vel_safe", 1);
  slow_poly_pub=nh.advertise<geometry_msgs::PolygonStamped>("/velocity_smoother/slow_polygon",1);
  stop_poly_pub=nh.advertise<geometry_msgs::PolygonStamped>("/velocity_smoother/stop_polygon",1);

  my_timer_ = nh.createTimer(ros::Duration(0.1), &VelocitySmoother::timerCallback, this);


  message_filters::Subscriber<spencer_tracking_msgs::DetectedPersons> spencer_sub(nh, "/spencer/perception/detected_persons", 1);
  message_filters::Subscriber<geometry_msgs::PolygonStamped> laser2_sub(nh, "/polygon", 1);
  typedef message_filters::sync_policies::ApproximateTime<spencer_tracking_msgs::DetectedPersons, geometry_msgs::PolygonStamped> MySyncPolicy;
   typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  sync.reset(new Sync(MySyncPolicy(10), spencer_sub, laser2_sub));
  sync->registerCallback(boost::bind(&VelocitySmoother::personCB_online,this, _1, _2));


  return true;
}
}
