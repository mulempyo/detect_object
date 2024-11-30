#include <detect_object/detect_object.h>
#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/Bool.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

namespace detect{

void Detect::subCostmap(costmap_2d::Costmap2DROS* costmap_ros){ //using local planner
    costmap_ros_ = costmap_ros;
}

void Detect::boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{

    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == "person" || box.Class == "pottedplant")
        {
            center_x = (box.xmax + box.xmin)/2;
            center_y = (box.ymax + box.ymin)/2;
            center_z = (box.zmax + box.zmin)/2;

            distance = std::sqrt(center_x * center_x + center_y * center_y + center_z * center_z);

            float obstacle_x = 5.608604;
            float obstacle_y = 1.296688;
            float obstacle_z = 0.18;

            float theta = std::atan2(center_y, center_x);

            geometry_msgs::PoseStamped current_pose_;
            costmap_ros_->getRobotPose(current_pose_);

            cameraToWorld(obstacle_x, obstacle_y, distance, theta, current_pose_.pose.position.x, current_pose_.pose.position.y);

            if(box.Class == "person" && box.probability >= 50){
             detected_person = true;
            }else{
              detected_person = false;
            }

            if(box.Class == "pottedplant"){
             detected_pottedplant = true;
            }else{
              detected_pottedplant = false;
            }

            detect_pottedplant_msg.data = detected_pottedplant;
            pub.publish(detect_pottedplant_msg);

            detected_person_msg.data = detected_person;
            pub1.publish(detected_person_msg);
        }
    }
  }

  void Detect::cameraToWorld(float obstacle_x, float obstacle_y, float distance, float theta, float robot_x, float robot_y){
    world_x = obstacle_x + distance * std::cos(theta);
    world_y = obstacle_y + distance * std::sin(theta);
    world_yaw = std::atan2(world_y - robot_y, world_x - robot_x);
    camera_yaw = world_yaw - theta;
  }

  float Detect::cameraX(){
    camera_x = world_x;
    return camera_x;
  }

  float Detect::cameraY(){
    camera_y = world_y;
    return camera_y;
  }

  float Detect::cameraYaw(){
    return camera_yaw;
  }
} //namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_boxes_3d");
    ros::NodeHandle nh;

    detect::Detect detector;
    
    detector.sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 10, &detect::Detect::boundingBoxCallback, &detector);
    detector.pub = nh.advertise<std_msgs::Bool>("3d_object_detection_true",10);
    detector.pub1 = nh.advertise<std_msgs::Bool>("person_probabilty",10);

    ros::spin();
    return 0;
}



