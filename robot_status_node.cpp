/*********************************************************
Copyright (C),2018,irobotics Co.,Ltd
----------------------------------------------------------
File name: robot_status_node.cpp
----------------------------------------------------------
Author: Zhengyi Min    Version:0.1       Date:2018.03.28
Email:154727924@qq.com & 18521515421@163.com
----------------------------------------------------------
Description:
----------------------------------------------------------
Main Function List:
----------------------------------------------------------
History:  0.1 first publishcation                       
            
  
**********************************************************/
#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <yocs_msgs/RobotStatus.h>
#include <std_msgs/String.h>
#include <yocs_msgs/NavigationControlStatus.h>
#include <yocs_msgs/NavigationControl.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>


using namespace std;

class robot_status_node
{
public:
    robot_status_node();
    ~robot_status_node();
 
private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    bool is_get_robot_status;
    bool was_moving_front ;
    yocs_msgs::RobotStatus robot_status;
    std_msgs::String previous_waypoint;
    std_msgs::String get_lifter_status;


    ros::Publisher  robot_status_pub_;
    ros::Publisher  waypoint_user_pub_;
    ros::Publisher  get_lifter_status_pub_;

    ros::Subscriber  get_robot_status_sub_;
    ros::Subscriber  nav_ctrl_status_sub_;
    ros::Subscriber  nav_ctrl_sub_;

    ros::Subscriber  cmd_vel_sub_;
    //ros::Subscriber  vol_capacity_sub_;

    ros::Subscriber  waypoint_user_sub_;
    ros::Subscriber  diagnostics_agg_sub_;

    //ros::Rate loop_rate(5.0);
      
    boost::mutex mutex_;
    
    std::string WARN_EMERGENCY;
    std::string WARN_OBSTACLE_FRONT;
    std::string WARN_OBSTACLE_BACK;
    std::string WARN_BATTERY_LOW;
    std::string WARN_WEIGHT_OVER;
    std::string WARN_HEIGHT_OVER;
    std::string WARN_WIDTH_OVER;
    std::string WARN_BARCODE_UNRECEIVED;
    std::string WARN_LOCATION_ERROR;
    std::string ERROR_SENSOR;
    std::string ERROR_TOUCH_SENSOR_FRONT;
    std::string ERROR_TOUCH_SENSOR_BACK;
    std::string ERROR_PX22_FRONT;
    std::string ERROR_PX22_BACK;
    std::string ERROR_PX24_FRONT;
    std::string ERROR_PX24_BACK;
    std::string ERROR_LASER_SENSOR_FRONT;
    std::string ERROR_LASER_SENSOR_BACK;
    std::string ERROR_BARCODE_READER;
    std::string ERROR_RFID_SENSOR;
    std::string ERROR_MAGNETIC_SENSOR_FRONT;
    std::string ERROR_MAGNETIC_SENSOR_BACK;
    std::string ERROR_IMU_SENSOR;
    std::string ERROR_MOTOR_DRIVER;
    std::string ERROR_MOTOR_DRIVER_FRONT;
    std::string ERROR_MOTOR_DRIVER_BACK;
    std::string ERROR_MOTOR_DRIVER_LEFT;
    std::string ERROR_MOTOR_DRIVER_RIGHT;
    std::string ERROR_MOTOR_DRIVER_FRONT_LEFT;
    std::string ERROR_MOTOR_DRIVER_FRONT_RIGHT;
    std::string ERROR_MOTOR_DRIVER_BACK_LEFT;
    std::string ERROR_MOTOR_DRIVER_BACK_RIGHT;
    std::string ERROR_LIFT_SYSTEM;
    std::string ERROR_LIFTER_FRONT;
    std::string ERROR_LIFTER_BACK;
    std::string ERROR_LIFTER_LEFT;
    std::string ERROR_LIFTER_RIGHT;
    std::string ERROR_LIFT_PLATFORM_LIFT;
    std::string ERROR_LIFT_PLATFORM_ROTATE;
    std::string ERROR_LIFT_PLATFORM_EXTEND;
    std::string ERROR_COMMUNICATION;
    std::string ERROR_COMMUNICATION_PLC;
    std::string ERROR_COMMUNICATION_CHARGER;
    std::string ERROR_COMMUNICATION_NETWORK;
    std::string ERROR_COMMUNICATION_DISPATCH;
    std::string INFO_MODE_NAVIGATION;
    std::string INFO_MODE_GMAPPING;    
 
private:
    void diagnostics_agg_sub(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    void waypoint_user_sub(const std_msgs::String::ConstPtr& msg);
    //void vol_capacity_sub(const std_msgs::Int8::ConstPtr& msg);
    void nav_ctrl_sub(const yocs_msgs::NavigationControl::ConstPtr& msg);
    void nav_ctrl_status_sub(const yocs_msgs::NavigationControlStatus::ConstPtr& msg);
    void cmd_vel_sub(const geometry_msgs::Twist::ConstPtr& msg);
    void get_robot_status_sub(const std_msgs::String::ConstPtr& msg);
    void do_action();

};


/*********************************************************************************
* FUNCTION NAME: robot_status_node()
* CREATE DATE : 2018/03/28
* CREATED BY : Zhengyi Min
* FUNCTION : Constructor
* MODIFY DATE :
* GLOBAL VAR: 
* INPUT : 
* OUTPUT : 
* RETURN : 
*          
*********************************************************************************/

robot_status_node::robot_status_node()
{
     ros::Rate rate(0.2);
     
     //get_robot_status = false;
     is_get_robot_status = false;
     was_moving_front = false;
     robot_status.action_number = "";
     robot_status.ctrl_status = "";
     robot_status.run_status = "";
     robot_status.is_at_waypoint = "";
     robot_status.towards_waypoint = "";
     robot_status.speed_level = "";
     robot_status.lifter_status = "";
     //robot_status.fault_number = "";
     previous_waypoint.data = "";
     get_lifter_status.data = "get_lifter_status";
         
     ph_.param("warn_emergency", WARN_EMERGENCY, std::string("1001"));
     ph_.param("warn_obstacle_front", WARN_OBSTACLE_FRONT, std::string("1011"));
     ph_.param("warn_obstacle_back", WARN_OBSTACLE_BACK, std::string("1012"));
     ph_.param("warn_battery_low", WARN_BATTERY_LOW, std::string("1003"));
     ph_.param("warn_weight_over", WARN_WEIGHT_OVER, std::string("1004"));
     ph_.param("warn_height_over", WARN_HEIGHT_OVER, std::string("1005"));
     ph_.param("warn_widht_over", WARN_WIDTH_OVER, std::string("1006"));
     ph_.param("warn_barcode_unreceived", WARN_BARCODE_UNRECEIVED, std::string("1007"));
     ph_.param("warn_location_error", WARN_LOCATION_ERROR, std::string("1008"));
     ph_.param("error_sensor", ERROR_SENSOR, std::string("2000"));
     ph_.param("error_touch_sensor_front", ERROR_TOUCH_SENSOR_FRONT, std::string("2011"));
     ph_.param("error_touch_sensor_back", ERROR_TOUCH_SENSOR_BACK, std::string("2012"));
     ph_.param("error_px22_front", ERROR_PX22_FRONT, std::string("2013"));
     ph_.param("error_px22_back", ERROR_PX22_BACK, std::string("2014"));
     ph_.param("error_px24_front", ERROR_PX24_FRONT,std::string("2015"));
     ph_.param("error_px24_back", ERROR_PX24_BACK,std::string("2016"));
     ph_.param("error_laser_sensor_front", ERROR_LASER_SENSOR_FRONT,std::string("2021"));
     ph_.param("error_laser_sensor_back", ERROR_LASER_SENSOR_BACK,std::string("2022"));
     ph_.param("error_barcode_reader", ERROR_BARCODE_READER,std::string("2030"));
     ph_.param("error_rfid_sensor", ERROR_RFID_SENSOR,std::string("2040"));
     ph_.param("error_magnetic_sensor_front", ERROR_MAGNETIC_SENSOR_FRONT,std::string("2051"));
     ph_.param("error_magnetic_sensor_back", ERROR_MAGNETIC_SENSOR_BACK,std::string("2052"));
     ph_.param("error_imu_sensor", ERROR_IMU_SENSOR,std::string("2060"));
     ph_.param("error_motor_driver", ERROR_MOTOR_DRIVER,std::string("3000"));
     ph_.param("error_motor_driver_front", ERROR_MOTOR_DRIVER_FRONT,std::string("3001"));
     ph_.param("error_motor_driver_back", ERROR_MOTOR_DRIVER_BACK,std::string("3002"));
     ph_.param("error_motor_driver_left", ERROR_MOTOR_DRIVER_LEFT,std::string("3003"));
     ph_.param("error_motor_driver_right", ERROR_MOTOR_DRIVER_RIGHT,std::string("3004"));
     ph_.param("error_motor_driver_front_left", ERROR_MOTOR_DRIVER_FRONT_LEFT,std::string("3005"));
     ph_.param("error_motor_driver_front_right", ERROR_MOTOR_DRIVER_FRONT_RIGHT,std::string("3006"));
     ph_.param("error_motor_driver_back_left", ERROR_MOTOR_DRIVER_BACK_LEFT,std::string("3007"));
     ph_.param("error_motor_driver_back_right", ERROR_MOTOR_DRIVER_BACK_RIGHT,std::string("3008"));
     ph_.param("error_lift_system", ERROR_LIFT_SYSTEM,std::string("4000"));
     ph_.param("error_lifter_front", ERROR_LIFTER_FRONT,std::string("4001"));
     ph_.param("error_lifter_back", ERROR_LIFTER_BACK,std::string("4002"));
     ph_.param("error_lifter_left", ERROR_LIFTER_LEFT,std::string("4003"));
     ph_.param("error_lifter_right", ERROR_LIFTER_RIGHT,std::string("4004"));
     ph_.param("error_lift_platform_front", ERROR_LIFT_PLATFORM_LIFT,std::string("4011"));
     ph_.param("error_lift_platform_rotate", ERROR_LIFT_PLATFORM_ROTATE,std::string("4012"));
     ph_.param("error_lift_platform_extend", ERROR_LIFT_PLATFORM_EXTEND,std::string("4013"));
     ph_.param("error_communication", ERROR_COMMUNICATION,std::string("5000"));
     ph_.param("error_communication_charger", ERROR_COMMUNICATION_CHARGER,std::string("5001"));
     ph_.param("error_communication_plc", ERROR_COMMUNICATION_PLC,std::string("5002"));
     ph_.param("error_communication_network", ERROR_COMMUNICATION_NETWORK,std::string("5003"));
     ph_.param("error_communication_dispatch", ERROR_COMMUNICATION_DISPATCH,std::string("5004"));
     ph_.param("info_mode_navigation", INFO_MODE_NAVIGATION,std::string("6001"));
     ph_.param("info_mode_gmapping", INFO_MODE_GMAPPING,std::string("6002"));
    



     robot_status_pub_ = nh_.advertise<yocs_msgs::RobotStatus>("receive_robot_status", 1, true);
     waypoint_user_pub_  = nh_.advertise<std_msgs::String>("waypoint_user_pub", 1, true);

     get_robot_status_sub_ = nh_.subscribe("get_robot_status",100, &robot_status_node::get_robot_status_sub, this);
     nav_ctrl_status_sub_ = nh_.subscribe("nav_ctrl_status",  1, &robot_status_node::nav_ctrl_status_sub, this);
     nav_ctrl_sub_ = nh_.subscribe("nav_ctrl",  1, &robot_status_node::nav_ctrl_sub, this);
     cmd_vel_sub_ = nh_.subscribe("cmd_vel",  1, &robot_status_node::cmd_vel_sub, this);
     //vol_capacity_sub_ = nh_.subscribe("vol_capacity",1,&robot_status_node::vol_capacity_sub,this);
     waypoint_user_sub_ = nh_.subscribe("waypoint_user_sub",1,&robot_status_node::waypoint_user_sub,this);
     diagnostics_agg_sub_ = nh_.subscribe("diagnostics_agg",1,&robot_status_node::diagnostics_agg_sub,this);
     
     while(ros::ok())
     {
        rate.sleep();
        ros::spinOnce();  
        do_action();
    }
}

robot_status_node::~robot_status_node()
{
   
}

void robot_status_node::do_action()
{
  
  // vector<boost::shared_ptr<std_msgs::String> > processed;
 //  {
   //  boost::mutex::scoped_lock lock(mutex_);
   //  processed = robot_status.fault_number ;     

     if(is_get_robot_status == true)
     {
       ROS_INFO("is_get_robot_status == true");
       waypoint_user_pub_.publish(get_lifter_status);
       robot_status_pub_.publish(robot_status);
     }
     else
     {
       ROS_ERROR("Waiting for order");
     }

  //  }

    robot_status.fault_number.clear();       
}

void robot_status_node::get_robot_status_sub(const std_msgs::String::ConstPtr& msg)
{
     //get_robot_status = true;
     //ROS_INFO("Front:I heard: [%s]", msg->data.c_str());
     std_msgs::String  get_robot_status;
     get_robot_status = *msg;
     if(get_robot_status.data == "get_robot_status")
     {
        is_get_robot_status = true;
     }
     else
     {
        is_get_robot_status = false;
     }    
     //ROS_INFO("Back:I heard: [%s]", msg->data.c_str());
     //ROS_INFO("is_get_robot_status is : %d\n",is_get_robot_status);
}

void robot_status_node::nav_ctrl_status_sub(const yocs_msgs::NavigationControlStatus::ConstPtr& msg)
{
     yocs_msgs::NavigationControlStatus  nav_ctrl_status;
     nav_ctrl_status = *msg;
     robot_status.action_number = nav_ctrl_status.waypoint_name;
     if(nav_ctrl_status.status == 3)
     {
        robot_status.ctrl_status = "1";
     }
     else if(nav_ctrl_status.status == 0)
     {
        robot_status.ctrl_status = "2";
     }
     else if(nav_ctrl_status.status == 1)
     {
        robot_status.ctrl_status = "3";
     }	

}

void robot_status_node::nav_ctrl_sub(const yocs_msgs::NavigationControl::ConstPtr& msg)
{
     yocs_msgs::NavigationControl  nav_ctrl;
     nav_ctrl = *msg;
     bool IS_AT_WAYPOINT = false;
     robot_status.towards_waypoint = nav_ctrl.goal_name;
     if(nav_ctrl.control == 0)
     {
     	robot_status.is_at_waypoint = "1";
     	previous_waypoint.data = nav_ctrl.goal_name;
     	robot_status.current_waypoint = nav_ctrl.goal_name; 	
     }
     else
     {
     	robot_status.is_at_waypoint = "0";
     	robot_status.current_waypoint = previous_waypoint.data;
     }
}

void robot_status_node::cmd_vel_sub(const geometry_msgs::Twist::ConstPtr& msg)
{	
    if(fabs(msg->linear.x) <= 0.3)
    {
       robot_status.speed_level = "1";
    }
    else if(0.3 < fabs(msg->linear.x) <= 0.5)
    {
       robot_status.speed_level = "2";
    }
    else if(fabs(msg->linear.x) > 0.5)
    {
       robot_status.speed_level = "3";
    }
    if(msg->linear.x > 0 )
    {
       robot_status.towards_direction = "1";
       was_moving_front = true;
    }
    else if(msg->linear.x < 0)
    {
       robot_status.towards_direction = "2";
       was_moving_front = false;
    }
    else
    {
       if(was_moving_front)
       {
          robot_status.towards_direction = "1";
       }
       else
       {
          robot_status.towards_direction = "2";
       }
    }
}

void robot_status_node::waypoint_user_sub(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String user_data_;
    bool pos = msg->data.find(':');
    if (pos)
    {
      // deal with warn exception (e.g. timeout)
        if (msg->data.substr(0, pos) == "warn" && msg->data.substr(pos + 1, msg->data.length() - pos - 1) == get_lifter_status.data) {
            robot_status.lifter_status = "error";
        }
        else if (msg->data.substr(0, pos) == get_lifter_status.data)
        {
            user_data_.data = msg->data.substr(pos + 1, msg->data.length() - pos - 1);
            if(user_data_.data == "01")
            {
               robot_status.lifter_status = "0";
            }
            else if(user_data_.data == "02")
            {
               robot_status.lifter_status = "1";

            }
            else
            {
               robot_status.fault_number.push_back(ERROR_LIFT_SYSTEM);//升降消故障
            }
        }
    }
    else
    {
            robot_status.fault_number.push_back("11");
        
    }
}

void robot_status_node::diagnostics_agg_sub(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    
    diagnostic_msgs::DiagnosticArray  diagnostic_array;
    diagnostic_msgs::DiagnosticStatus  diagnostic_status;
    diagnostic_array = *msg;
   //vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed;
   //{
    // boost::mutex::scoped_lock lock(mutex_);
    // processed = msg->status;
  // }

   for (unsigned int i = 0; i < diagnostic_array.status.size(); ++i)
   {
      diagnostic_status = diagnostic_array.status[i];
     //if (processed[i]->level = 2)
     if (diagnostic_status.level = 2)
     {
         //robot_status.fault_number = processed[i]->hardware_id;
         robot_status.fault_number.push_back(ERROR_SENSOR);//传感器故障
     }
     else
     {
        robot_status.fault_number.push_back("0");

        if (diagnostic_status.level = 1)
        {
          if(diagnostic_status.name == "voltage")
          {
            robot_status.vol_capacity = diagnostic_status.message.substr(diagnostic_status.message.size()-2,diagnostic_status.message.size());
          }
        }
        else
        {
         ROS_INFO("diagnostic_status.level is : %d\n",diagnostic_status.level);  
        }
     } 
   }
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "robot_status_node");
   robot_status_node robot_status_node;
   ros::spin();
}

















