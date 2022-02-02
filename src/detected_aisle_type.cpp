#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "intersection_recognition/Hypothesis.h"
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        void getRosParam(void);
	void detected_aisle_type(const intersection_recognition::Hypothesis::ConstPtr& hypothesis);

     private:
        ros::NodeHandle node_;
        ros::Subscriber hypothesis_sub_;
	ros::Publisher aisle_type_pub_;
   };

cmdVelController::cmdVelController(){
    hypothesis_sub_ = node_.subscribe<intersection_recognition::Hypothesis> ("hypothesis", 1, &cmdVelController::detected_aisle_type, this);
    
    aisle_type_pub_ = node_.advertise<std_msgs::String> ("aisle_type", 1);
}

void cmdVelController::detected_aisle_type(const intersection_recognition::Hypothesis::ConstPtr& hypothesis){
    std_msgs::String aisle_type;
    // check "straight_road"
    if(hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && !hypothesis->right_flg){
	    aisle_type.data = "straight_road";
    }


// check 3_way_left and 3_way_right when 3_way is designated by scenario
    // 3_way_left
    else if(hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && !hypothesis->right_flg){
            aisle_type.data = "3_way_left";
    }
    // 3_way_right
    else if(hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && hypothesis->right_flg){
            aisle_type.data = "3_way_right";
    }
     // 3_way_center
    else if(!hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && hypothesis->right_flg){
            aisle_type.data = "3_way_center";
    }

// check "end"(= 突き当り)
    // dead_end
    else if(!hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && !hypothesis->right_flg){
            aisle_type.data = "end";
    }
    // right
    else if(!hypothesis->center_flg && hypothesis->back_flg && !hypothesis->left_flg && hypothesis->right_flg){
            aisle_type.data = "right";
    }
    // left
    else if(!hypothesis->center_flg && hypothesis->back_flg && hypothesis->left_flg && !hypothesis->right_flg){
            aisle_type.data = "left";
    }
    else{
    }

    aisle_type_pub_.publish(aisle_type);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "detected_aisle_type");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
