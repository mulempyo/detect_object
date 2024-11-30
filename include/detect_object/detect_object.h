#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs::Bool.h>
#include <gd_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

namespace detect{
    class Detect{
        public:
      
        ros::Publisher pub;
        ros::Publisher pub1;
        ros::Subscriber sub;

        std_msgs::Bool detect_pottedplant_msg;
        std_msgs::Bool detected_person_msg;

        bool detected_person = false;
        bool detected_pottedplant = false;

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

        void boundingBoxCallback(const gd_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg);
        void subCostmap(costmap_2d::Costmap2DROS* costmap_ros);
        
        float camera_x();
        float camera_y();
        float camera_yaw();

        private:
            void cameraToWorld(float obstacle_x, float obstacle_y, float distance, float theta);
    }
}

#endif
