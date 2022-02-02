#include "ros/ros.h"
#include "std_msgs/String.h"
#include "intersection_recognition/Hypothesis.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <yolov5_pytorch_ros/BoundingBox.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <cstdlib>
#include <cmath>
#include <bits/stdc++.h>
#include <vector>

class intersectionRecognition {
    public:
        intersectionRecognition();
        double door_size_thresh;
	double probability_thresh;
	int y_min_thresh;
	int y_max_thresh;
	int x_left_min_thresh;
    	int x_left_max_thresh;
    	int x_center_min_thresh;
    	int x_center_max_thresh;
    	int x_right_min_thresh;
    	int x_right_max_thresh;
    	int x_back_min_thresh;
    	int x_back_max_thresh;
        void get_ros_param(void);
        intersection_recognition::Hypothesis generate_publish_variable(
        	bool center_flg, bool back_flg, bool left_flg, bool right_flg 
        );
        void BBCallback(
        	const yolov5_pytorch_ros::BoundingBoxesConstPtr& boundingboxes
        );

    private:
        ros::NodeHandle node_;
        ros::Subscriber boundingboxes_sub_;
	ros::Publisher hypothesis_pub_;
};

intersectionRecognition::intersectionRecognition()
{
    boundingboxes_sub_ = node_.subscribe<yolov5_pytorch_ros::BoundingBoxes>("detected_objects_in_image", 1, &intersectionRecognition::BBCallback, this);
    hypothesis_pub_ = node_.advertise<intersection_recognition::Hypothesis>("hypothesis", 1);
}

void intersectionRecognition::get_ros_param(void){
    door_size_thresh = 0.5;
	probability_thresh = 0.5;
    y_min_thresh = 130;
    y_max_thresh = 230;
    x_left_min_thresh = 40;
    x_left_max_thresh = 140;
    x_center_min_thresh = 190;
    x_center_max_thresh = 290;
    x_right_min_thresh = 350;
    x_right_max_thresh = 450;
    x_back_min_thresh = 500;
    x_back_max_thresh = 600;
    node_.getParam("extended_toe_finding/door_size_thresh", door_size_thresh);
    node_.getParam("extended_toe_finding/probability_thresh", probability_thresh);
    node_.getParam("y_min_thresh", y_min_thresh);
    node_.getParam("y_max_thresh", y_max_thresh);
    node_.getParam("x_left_min_thresh", x_left_min_thresh);
    node_.getParam("x_left_max_thresh", x_left_max_thresh);
    node_.getParam("x_center_min_thresh", x_center_min_thresh);
    node_.getParam("x_center_max_thresh", x_center_max_thresh);
    node_.getParam("x_right_min_thresh", x_right_min_thresh);
    node_.getParam("x_right_max_thresh", x_right_max_thresh);
    node_.getParam("x_back_min_thresh", x_back_min_thresh);
    node_.getParam("x_back_max_thresh", x_back_max_thresh);
}

void intersectionRecognition::BBCallback(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& boundingboxes){
	std::vector<int> tentative_hyp(4, 0);
	std::vector<int> pre_tentative_hyp(4, 0);
	std::vector<yolov5_pytorch_ros::BoundingBox> yolo_result = boundingboxes->bounding_boxes;
	intersection_recognition::Hypothesis hypothesis;
	for(const auto obj : yolo_result){ //per box
		double obj_size, x_center, y_center;
		if(obj.Class == "aisle" && obj.probability >= probability_thresh){ //only aisle & above thresh
			obj_size = obj.xmax - obj.xmin;
			if(obj_size >= door_size_thresh){
				x_center = (obj.xmin + obj.xmax)/2;
				y_center = (obj.ymin + obj.ymax)/2;

				if((x_left_min_thresh <= x_center && x_center <= x_left_max_thresh) && (y_min_thresh <= y_center && y_center <= y_max_thresh)){
					tentative_hyp[0] = 1;
				}
				else{
					tentative_hyp[0] = 0;
				}
				if((x_center_min_thresh <= x_center && x_center <= x_center_max_thresh) && (y_min_thresh <= y_center && y_center <= y_max_thresh)){
					tentative_hyp[1] = 1;
				}	
				else{
					tentative_hyp[1] = 0;
				}
				if((x_right_min_thresh <= x_center && x_center <= x_right_max_thresh) && (y_min_thresh <= y_center && y_center <= y_max_thresh)){
					tentative_hyp[2] = 1;
				}
				else{
					tentative_hyp[2] = 0;
				}
				if((x_back_min_thresh <= x_center && x_center <= x_back_max_thresh) && (y_min_thresh <= y_center && y_center <= y_max_thresh)){
					tentative_hyp[3] = 1;
				}
				else{
					tentative_hyp[3] = 0;
				}
				for(int i = 0; i < tentative_hyp.size(); i++){
					if(tentative_hyp[i] == 1 || pre_tentative_hyp[i] == 1){
						pre_tentative_hyp[i] = 1;
					}
				}
			}
			else{
				for(int i = 0; i < tentative_hyp.size(); i++){
					tentative_hyp[i] = 0;
				}
				for(int i = 0; i < tentative_hyp.size(); i++){
					if(tentative_hyp[i] == 1 || pre_tentative_hyp[i] == 1){
						pre_tentative_hyp[i] = 1;
					}
				}
			}
		}
		else{
			for(int i = 0; i < tentative_hyp.size(); i++){
				tentative_hyp[i] = 0;
			}
			for(int i = 0; i < tentative_hyp.size(); i++){
				if(tentative_hyp[i] == 1 || pre_tentative_hyp[i] == 1){
					pre_tentative_hyp[i] = 1;
				}
			}
		}
	}
	if(pre_tentative_hyp[0] == 1){
		hypothesis.left_flg = true;
	}
	else{
		hypothesis.left_flg = false;
	}
	if(pre_tentative_hyp[1] == 1){
		hypothesis.center_flg = true;
	}
	else{
		hypothesis.center_flg = false;
	}
	if(pre_tentative_hyp[2] == 1){
		hypothesis.right_flg = true;
	}
	else{
		hypothesis.right_flg = false;
	}
	if(pre_tentative_hyp[3] == 1){
		hypothesis.back_flg = true;
	}
	else{
		hypothesis.back_flg = false;
	}
	hypothesis_pub_.publish(hypothesis);	
}


int main(int argc, char** argv){
    ros::init(argc, argv, "extended_toe_finding");
    intersectionRecognition recognition;
    recognition.get_ros_param();
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}
