#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "intersection_recognition/Scenario.h"
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        geometry_msgs::Twist vel_;
        double IMU_HZ = 100.0;
        double CHANGE_DIRECTION_RAD = 0.0;
        float reverse_turn = 0;
        float rotate_rad_ = 0;
        void getRosParam(void);
        void moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad);
        void emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg);

     private:
        ros::NodeHandle node_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher turn_finish_flg_pub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber rotate_rad_sub_;
        ros::Subscriber emergency_stop_flg_sub_;

        bool turn_flg_ = false;
        bool emergency_stop_flg_ = true;
        const int SCENARIO_MAX = 10;
        std::string last_node_ = "start";
        double target_yaw_rad_ = 0;
        double current_yaw_rad_ = 0;
};

cmdVelController::cmdVelController(){
    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
    turn_finish_flg_pub_ = node_.advertise<std_msgs::Bool>("turn_finish_flg", 1, false);

    imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("imu_data", 1, &cmdVelController::moveCallback, this);
    rotate_rad_sub_ = node_.subscribe<std_msgs::Float32> ("rotate_rad", 1, &cmdVelController::turnRadCallback, this);
    emergency_stop_flg_sub_ = node_.subscribe<std_msgs::Bool> ("emergency_stop_flg", 1, &cmdVelController::emergencyStopFlgCallback, this);

    getRosParam();
}

void cmdVelController::getRosParam(void){
    IMU_HZ = 100.0;
    reverse_turn = 1.0;
    node_.getParam("cmd_vel_controller/IMU_HZ", IMU_HZ);
    node_.getParam("cmd_vel_controller/reverse_turn", reverse_turn);
}

void cmdVelController::moveCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    current_yaw_rad_ += imu_data->angular_velocity.z / IMU_HZ;
    if(! emergency_stop_flg_){
        if(turn_flg_){
        // rotate_rad += imu_data->angular_velocity.z[rad/sec] * (1/IMU_HZ)[sec]
            rotate_rad_ += imu_data->angular_velocity.z / IMU_HZ;
        // 3.14/180 means 1[rad]
            std::cout << "rotate rad is " << rotate_rad_ << std::endl;
            std::cout << "target - current is " <<  -(target_yaw_rad_ - current_yaw_rad_) << std::endl;
            if(std::abs(rotate_rad_) < 3.14/180){
                turn_flg_ = false;
                std_msgs::Bool turn_finish_flg_for_pub;
                turn_finish_flg_for_pub.data = true;
                turn_finish_flg_pub_.publish(turn_finish_flg_for_pub);
                rotate_rad_ = 0;
            }

        // if rotate_rad = 0, do nothing because it is turn_flg is false.
            if(rotate_rad_ < 0){
                vel_.angular.z = -0.5;
            }
            else if(rotate_rad_ > 0){
                vel_.angular.z = 0.5;
            }

            vel_.linear.x = 0.0;
            cmd_vel_pub_.publish(vel_);
            vel_.angular.z = 0.0;
        }
    // if turn_flg is false
        else{
            vel_.linear.x = 0.55;
        // vel_.angular.z = (target_yaw_rad_ - current_yaw_rad_)[rad] * (1/IMU_HZ)[sec]
            vel_.angular.z = -(target_yaw_rad_ - current_yaw_rad_) * reverse_turn;
            cmd_vel_pub_.publish(vel_);
            vel_.linear.x = 0.0;
        }
    }
    else{
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
        cmd_vel_pub_.publish(vel_);
    }
}


void cmdVelController::turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad){
    rotate_rad_ = turn_rad->data * reverse_turn;
    target_yaw_rad_ -= turn_rad->data * reverse_turn;
    if(rotate_rad_ == 0.0){
        turn_flg_ = false;
    }
    else{
        turn_flg_ = true;
    }
}

void cmdVelController::emergencyStopFlgCallback(const std_msgs::Bool::ConstPtr& emergency_stop_flg){
    emergency_stop_flg_ = emergency_stop_flg->data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_controller");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
