#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <string>
#include <cmath>

namespace smb_highlevel_controller
{
    class SmbHighlevelController
    {
        public: 
            // Constructor
            SmbHighlevelController(ros::NodeHandle& nh);

            // Copy constructor
            // defines ad DELETES for simplicity
            SmbHighlevelController(const SmbHighlevelController &) = delete;

            // no assignment = 
            SmbHighlevelController& operator=(const SmbHighlevelController &) = delete;
            
            // Destructor
            ~SmbHighlevelController() = default;
            
            // set Smb velocity
            // set robot's linear & angular velocity
            void setVel(const float vel, const std::string &dof);
            
            // ROS topic publish function
            // publish a message to topic /cml_vel to send a Twist to the robot
            void DriveSmb();

            // adjust robot forward speed
            // adjust speed using saturated P control
            void adjustSpeed(const float dist);

            // adjust robot heading
            // adjust heading using P control
            void adjustHeading(const float ang);

            // visualize pillar with marker in RViz
            void vizPillar();

        private:
            // data
            ros::NodeHandle nodeHandle;
            ros::Subscriber subscriber;
            ros::Publisher vel_pub, viz_pub;
            ros::ServiceServer stop_srv;
            geometry_msgs::Twist msg;
            visualization_msgs::Marker marker;
            float p_ang, p_vel;
            float pillar_pos[2];

            bool emergency_stop = true;

            // ROS topic callback function
            // print out the position of the pillar with respect to the robot
            // and adjust the robot heading towards the pillar
            void LaserCallback(const sensor_msgs::LaserScan &msg);

            // ROS service callback function start/stop the robot
            bool start_stop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

            // initialize pillar marker in RViz
            void initPillarMarker();
    };
} // namespace smb_highlevel_controller
