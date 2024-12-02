#include <detect_object/detect_object.h>
#include "ros/ros.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/Float64.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

namespace detect{

Detect::Detect(){
  tf_.reset(new tf2_ros::Buffer);
  tf_->setUsingDedicatedThread(true);
  tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

Detect::~Detect(){
   
}

void Detect::boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == "pottedplant")
        {
            center_x = (box.xmax + box.xmin)/2;
            center_y = (box.ymax + box.ymin)/2;
            center_z = (box.zmax + box.zmin)/2;

            distance = std::sqrt(center_x * center_x + center_y * center_y + center_z * center_z);

            float obstacle_x = 5.608604;
            float obstacle_y = 1.296688;
            float obstacle_z = 0.18;

            float theta = std::atan2(center_y, center_x);

            cameraToWorld(obstacle_x, obstacle_y, distance, theta);
                
            pottedplant.data = 1.0;
            ROS_WARN("detect_pottedplant_msg.data:%f",pottedplant.data);
            pub.publish(pottedplant); 
        }
    }
   person.data = 1.0;
   pub1.publish(person);
  }

  void Detect::cameraToWorld(float obstacle_x, float obstacle_y, float distance, float theta){
    world_x = obstacle_x + distance * std::cos(theta);
    world_y = obstacle_y + distance * std::sin(theta);

    geometry_msgs::PoseStamped camera_world;
    camera_world.header.frame_id = "camera_link";
    camera_world.header.stamp = ros::Time(0);
    camera_world.pose.position.x = world_x;
    camera_world.pose.position.y = world_y;

    geometry_msgs::PoseStamped robot;
    geometry_msgs::TransformStamped transformStamped = tf_->lookupTransform("base_footprint", "camera_link", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(camera_world, robot, transformStamped);

    world_yaw = std::atan2(world_y - robot.pose.position.y, world_x - robot.pose.position.x);
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
    detector.pub = nh.advertise<std_msgs::Float64>("object_detection_true",10);
    detector.pub1 = nh.advertise<std_msgs::Float64>("person_probabilty",10);

    ros::spin();
    return 0;
}



