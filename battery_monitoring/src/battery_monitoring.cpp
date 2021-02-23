

#include "battery_monitoring/battery_monitoring.h"

void BatteryMonitoring::resetCounters()
{
    counter_charging = 0;
    sum_voltage_charging = 0;
    sum_voltage_discharging = 0;
    counter_discharging = 0;
}

//########## CONSTRUCTOR ###############################################################################################
BatteryMonitoring::BatteryMonitoring(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // === PARAMETERS ===
    node_->param("battery_monitoring/voltage_full", voltage_full, 27.45);
    node_->param("battery_monitoring/voltage_start_recharging", voltage_start_recharging, 23.60);
    node_->param("battery_monitoring/recharge_time", recharge_time, 12000.00);
    node_->param("battery_monitoring/recharge_time_night", recharge_time_night, 50400.00);
    node_->param("battery_monitoring/start_hour", start_hour, 19);

    // === SUBSCRIBERS ===
    battery_state_subscriber_ = node_->subscribe("/base/relayboard_v2/battery_state", 10, &BatteryMonitoring::subscriberCallback, this);
    state_subscriber_ = node_->subscribe("/base/relayboard_v2/state", 10, &BatteryMonitoring::statesubscriberCallback, this);

    // === PUBLISHERS ===
    battery_monitoring_publisher_ = node_->advertise<cmr_msgs::BatteryMonitoring>("/battery_monitoring/battery_charge_state", 10);

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.03), &BatteryMonitoring::timerCallback, this);

    still_electrical_contact = false;
    charging = false;
    full = false;
    recharge_needed = false;

    ros::Duration tmmax(recharge_time);
    timeout = tmmax;
    resetCounters();
}

//########## CALLBACK: SUBSCRIBER ######################################################################################
void BatteryMonitoring::subscriberCallback(const sensor_msgs::BatteryStateConstPtr& battery_state)
{
    //http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html
     battery_charge_state = battery_state;
}

void BatteryMonitoring::statesubscriberCallback(const neo_msgs::RelayBoardV2ConstPtr& state)
{
    //http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html
    base_state = state;
}

//########## CALLBACK: TIMER ###########################################################################################
void BatteryMonitoring::timerCallback(const ros::TimerEvent &evt)
{
    if(battery_charge_state != NULL)
    {
        if(battery_charge_state->power_supply_status == 1 && charging == false)
        {
            charging = true;
            start_time = ros::Time::now();
            resetCounters();
            print_status = true;
        }
        if(charging == true)
        {
           //for the special case that the robot is already charging because of low voltage and the night charging mode turns on.
           //https://www.tutorialspoint.com/cplusplus/cpp_date_time.htm
           //https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
           std::time_t t = std::time(0);   // get time now
           std::tm* now = std::localtime(&t);
           ros::Duration tmmax(recharge_time);
           ros::Duration tmnight(recharge_time_night);

           //ROS_INFO("%d:%d:%d",now->tm_hour,now->tm_min,now->tm_sec);

           if(start_hour == now->tm_hour && timeout == tmmax)
           {
               start_time = ros::Time::now();
               timeout = tmnight;
           }

           sum_voltage_charging += battery_charge_state->voltage;
           counter_charging += 1;
           if(counter_charging > 20)
           {
               //first condition: cahrge for a specific duration, for example during the night for 12 hours for nightly charging, otherwise different timeout for charging
               //second condition: charge until max voltage reached, mustn't be used during nightly charging! so timeout mustn't be tmnight
               //third condition: while still electrical contact, keep sending the output_battery_charge_state message with recharge_needed = false, otherwise problems occur during undocking
               //fourth condition: check if daytime, at night robot will always charge
               if(((ros::Time::now() - start_time > timeout) || (base_state->charging_state == 7) && (timeout == tmmax) || still_electrical_contact)&&(now->tm_hour > 8 && now->tm_hour < 19))
               {
                   charging = false;
                   still_electrical_contact = true;
                   output_battery_charge_state.full = true;
                   output_battery_charge_state.recharge_needed = false;
                   //output_battery_charge_state.charging = true;
                   battery_monitoring_publisher_.publish(output_battery_charge_state);
                   if(ros::Time::now() - start_time > timeout)
                       ROS_INFO("timeout");
                   else if(base_state->charging_state == 7)
                       ROS_INFO("Max voltage reached.");
                   resetCounters();
                   print_status = true;
               }else
               {
                   resetCounters();
               }
           }
        }else
        {
           //https://www.tutorialspoint.com/cplusplus/cpp_date_time.htm
           //https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
           std::time_t t = std::time(0);   // get time now
           std::tm* now = std::localtime(&t);

           still_electrical_contact = false;
           sum_voltage_discharging += battery_charge_state->voltage;
           counter_discharging += 1;
           //ROS_INFO("counter_discharging: %d", counter_discharging);
           //ROS_INFO("sum_voltage_discharging: %f", sum_voltage_discharging);
           //ROS_INFO("ask_voltage: %f", sum_voltage_discharging/counter_discharging);
           if(counter_discharging > 20)
           {
               ROS_INFO("%d",start_hour);
               if(sum_voltage_discharging/counter_discharging <= voltage_start_recharging || start_hour == now->tm_hour )
               {
                   output_battery_charge_state.full = false;
                   output_battery_charge_state.recharge_needed = true;
                   //output_battery_charge_state.charging = false;
                   battery_monitoring_publisher_.publish(output_battery_charge_state);
                   if(sum_voltage_discharging/counter_discharging <= voltage_start_recharging)
                   {
                        ros::Duration tmmax(recharge_time);
                        timeout = tmmax;
                        if(print_status ==true)
                        {
                            ROS_INFO("Low battery voltage, going to charging station. %d", timeout.sec);
                            print_status = false;
                        }
                   }
                   else
                   {
                       ros::Duration tmnight(recharge_time_night);
                       timeout = tmnight;
                       if(print_status ==true)
                       {
                           ROS_INFO("Time for nightly charging. %d", timeout.sec);
                           print_status = false;
                       }
                   }
                   resetCounters();
               }else if(sum_voltage_discharging/counter_discharging > (voltage_start_recharging + 0.30))
               {
                   output_battery_charge_state.full = false;
                   output_battery_charge_state.recharge_needed = false;
                   //output_battery_charge_state.charging = false;
                   battery_monitoring_publisher_.publish(output_battery_charge_state);
                   resetCounters();
               }
               resetCounters();
           }
        }
    }else
        ROS_INFO("Subscriber did not receive any data.");
        return;
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_monitoring");

    ros::NodeHandle node_handle;
    BatteryMonitoring battery_monitoring(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
