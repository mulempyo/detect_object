#include <dect_object/detect_object.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs::Float64.h>
#include <std_msgs::Bool.h>
#include <gd_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

namespace detect{

void Detect::initialize(ros::NodeHandle& nh){
    sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 10, boundingBoxCallback);
    pub = nh.advertise<std_msgs::Float64>("3d_detection_object_distance",10);
    pub1 = nh.advertise<std_msgs::Bool>("person_probabilty",10);
}

void Detect::subCostmap(costmap_2d::Costmap2DROS* costmap_ros){ //using local planner
    costmap_ros_ = costmap_ros;
}

void Detect::boundingBoxCallback(const gd_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{

    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == "person" || box.Class == "chair" || box.Class == "pottedplant")
        {
            center_x = (box.xmax + box.xmin)/2;
            center_y = (box.ymax + box.ymin)/2;
            center_z = (box.zmax + box.zmin)/2;

            distance = std::sqrt(center_x * center_x + center_y * center_y + center_z * center_z);

            geometry_msgs::PointStamped current_pose_;
            costmap_ros_->getRobotPose(current_pose_);

            cameraToWorld(center_x, center_y, current_pose_.pose.position.x, current_pose_.pose.position.y, 
                        tf2::getYaw(robot_vel_tf.pose.orientation));

            if(box.Class == "person" && box.probability >= 50){
             detected_person = true;
            }

            distance_msg.data = distance;
            pub.publish(distance_msg);

            detected_person_msg.data = detected_person;
            pub1.publish(detected_person_msg);
        }
    }
  }

  void Detect::cameraToWorld(double center_x, double camera_y, double robot_pose_x, double robot_pose_y, double robot_pose_theta){
    world_x = robot_pose_x + (center_x * std::cos(robot_pose_theta) - center_y * std::sin(robot_pose_theta));
    world_y = robot_pose_y + (center_x * std::sin(robot_pose_theta) + center_y * std::cos(robot_pose_theta));
  }

  float Detect::camera_x(){
    camera_x = world_x;
    return camera_x;
  }

  float Detect::camera_y(){
    camera_y = world_y;
    return camera_y;
  }
} //namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_boxes_3d_distance");
    ros::NodeHandle nh;
    
    detect::Detect detector;
    detector.initialize(nh);
    ros::spin();
    return 0;
}



