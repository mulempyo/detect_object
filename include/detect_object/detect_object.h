#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

namespace detect{
    class Detect{
        public:
         Detect();
         ~Detect();

        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub1;

        std_msgs::Float64 pottedplant;
        std_msgs::Float64 person;

        float center_x;
        float center_y;
        float center_z;
        float distance;
        float theta_x;
        float theta_y;
        float camera_x;
        float camera_y;
        float camera_yaw;
        float world_x, world_y, world_yaw;

        void boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg);
        
        float cameraX();
        float cameraY();
        float cameraYaw();

        private:
           std::shared_ptr<tf2_ros::Buffer> tf_;
           std::shared_ptr<tf2_ros::TransformListener> tfl_;
            void cameraToWorld(float obstacle_x, float obstacle_y, float distance, float theta);
    };
}

#endif
