#include "smb_highlevel_controller/SmbHighlevelController.hpp"

namespace smb_highlevel_controller {
    
// Constructor
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nh) 
    :nodeHandle(nh), subscriber(), vel_pub(), viz_pub(), marker(), msg(), p_ang(100), p_vel(0.3), stop_srv()
    {
        // get param from config file
        nodeHandle.getParam("controller_gain", p_vel);
        std::string topic;
        int queue_size;
        std::string service;
        if(!nodeHandle.getParam("scan_topic", topic) || !nodeHandle.getParam("queue_size", queue_size) || !nodeHandle.getParam("service", service))
        {
            ROS_ERROR("Could not find subscriber params!");
            ros::requestShutdown();
        }

        // create subscriber
        subscriber = nodeHandle.subscribe(topic, queue_size, &SmbHighlevelController::LaserCallback, this);

        // create publisher
        vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        viz_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        // service server
        stop_srv = nodeHandle.advertiseService(service, &SmbHighlevelController::start_stop, this);

        // pillar marker in RViz
        initPillarMarker();
        ROS_INFO("Smb highlevel controller node launched!");

    }


// set Smb velocity
// set robot's linear & angular velocity
void SmbHighlevelController::setVel(const float vel, const std::string &dof)
{
    if (dof == "linear")
    {
        msg.linear.x = vel;
    }
    else if (dof == "angular")
    {
        msg.angular.z = vel;
    }
}

// ROS topic publish function
// publish a message to topic /cmd_vel to send a Twist to the robot
void SmbHighlevelController::DriveSmb()
{
    vel_pub.publish(msg);
}

// adjust robot forward speed
// adjust speed using saturated P control
void SmbHighlevelController::adjustSpeed(const float dist)
{
    setVel(5.0, "linear");
}

// adjust robot heading
// adjust heading using P control
void SmbHighlevelController::adjustHeading(const float ang)
{
    float diff = ang;
    setVel(p_ang * diff, "angular");
}

// visualize pillar with marker in RViz 
void SmbHighlevelController::vizPillar() 
{
    marker.pose.position.x = pillar_pos[0];
    marker.pose.position.y = pillar_pos[1];
    marker.pose.position.z = -1.0;
    viz_pub.publish(marker);
}  

// Initialize pillar marker in RViz
void SmbHighlevelController::initPillarMarker() 
{
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "smb_highlevel_controller";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pillar_pos[0];
    marker.pose.position.y = pillar_pos[1];
    marker.pose.position.z = -1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

// ROS topic callback function
// print out the position of the pillar with respect to the robot
// and adjust the robot heading towards the pillar
void SmbHighlevelController::LaserCallback(const sensor_msgs::LaserScan &msg)
{
    // first get the distance
    std::int32_t min_index = 0;
    float dist = msg.ranges[min_index];
    for (std::size_t i = 0; i < msg.ranges.size(); i++)
    {
        if(dist > msg.ranges[i])
        {
            dist = msg.ranges[i];
            min_index = i;
        }
    }

    // then get the sensor angle
    float range_resolution = (msg.angle_max - msg.angle_min) / msg.ranges.size();
    float ang = range_resolution * min_index + msg.angle_min; //rad
    // ROS_INFO_STREAM("\n\nKhoang cach toi Pillar la: " << "\nDistance: " << dist << " m" 
    // << "\nGoc toi Pillar la: " << "\nAngle: " << ang / M_PI * 180.0 << " degrees\n");

    // calculate the coordinate
    pillar_pos[0] = dist * std::cos(ang);
    pillar_pos[1] = dist * std::sin(ang);
    // ROS_INFO_STREAM("\n\nToa do Pillar: " << "\nx = " << pillar_pos[0] << "\ny = " << pillar_pos[1] << "\n");

    // viz pillar
    vizPillar();

    // adjust heading & drive Smb
    if(!emergency_stop){
        adjustSpeed(dist);
        adjustHeading(ang);
        DriveSmb();
    }
}

//ROS service callback function start/stop the robot
bool SmbHighlevelController::start_stop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
    emergency_stop = request.data;
    if (emergency_stop) { // stop
        response.message = "Stop Smb";
        setVel(0.0, "linear");
        setVel(0.0, "angular");
        DriveSmb();
    }
    else{ // start
        response.message = "Start Smb";
    }
    ROS_INFO_STREAM(response.message);
    response.success = true;
    return true;
}

}